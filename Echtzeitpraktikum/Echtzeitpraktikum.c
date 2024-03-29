#include <rtai_mbx.h>
#include <rtai_sched.h>
#include <rtai_sem.h>
#include <uint128.h>
#include <sys/rtai_modbus.h>

#define STACKSIZE 10000

MODULE_LICENSE("GPL");

// sensoren belegung
#define IN_DREHTELLER_SENSOR (1 << 0)
#define IN_BOHRER (1 << 1)
#define IN_PRUEFER (1 << 2)
#define BOHRER_OBEN (1 << 3)
#define BOHRER_UNTEN (1 << 4)
#define DREHTELLER_IN_POS (1 << 5)
#define PRUEFER_NORMALLAGE (1 << 6)

// aktoren
#define BOHRER_AN (1 << 0)
#define DREHTELLER_AN (1 << 1)
#define BOHRER_RUNTER (1 << 2)
#define BOHRER_HOCH (1 << 3)
#define BOHRER_FESTHALTEN (1 << 4)
#define PRUEFER_AN (1 << 5)
#define AUSWERFER_AUSGANG (1 << 6)
#define AUSWERFER_EINGANG (1 << 7)

static RT_TASK task_control;
static RT_TASK task_drehteller;
static RT_TASK task_pruefer;
static RT_TASK task_bohrer;
static RT_TASK task_ausgang;

static int fd_node;
static SEM sem;
static int MBX_COUNT = 5;
static MBX mbox[5];

// teile array auf dem drehteller
static int teile_pos[4];

// mbox->tasks messages
#define control_mbox 0
#define drehteller_mbox 1
#define pruefer_mbox 2
#define bohrer_mbox 3
#define ausgang_mbox 4

// tasks->mbox messages
#define mbox_drehteller_in_pos 0
#define mbox_pruefer_right 1
#define mbox_pruefer_wrong 2
#define mbox_bohrer 3
#define mbox_ausgang 4

#define type_set 1
#define type_rem 0

static int maskAllBits(short mask, int masktype) {
	short val;

	// semaphore
	rt_sem_wait(&sem);

	// alten wert auslesen
	rt_modbus_get(fd_node, DIGITAL_OUT, 0, &val);

	// alten wert ergänzen
	if (masktype == type_set)
		val |= mask;
	else if (masktype == type_rem)
		val &= ~mask;

	// neuen wert setzen
	rt_modbus_set(fd_node, DIGITAL_OUT, 0, val);

	/* Weiteres Problem: Wird nach dem Schreiben der Ausgaenge der Zustand der
	 * Ausgaenge eher wieder eingelesen, als die Ausgaenge physikalisch aktiv
	 * sind, werden falsche Werte gelesen. Deshalb muss an dieser Stelle
	 * gewartet werden, bis die Ausgaenge stabil sind. Wie lange? Hmm, die
	 * Modbus-Doku sagt nix, also ausprobieren!
	 */
	rt_sleep(1 * nano2count(1000000));

	// sem freigeben
	rt_sem_signal(&sem);

	return 0;
}

static void initProgram(void) {
	short val;
	int cmd, i;

	// array auf 0 setzen (keine teile auf dem teller)
	teile_pos[0] = 0;
	teile_pos[1] = 0;
	teile_pos[2] = 0;
	teile_pos[3] = 0;

	// bohrer auf start position setzten
	// festhalter zurücksetzen
	maskAllBits(BOHRER_HOCH, type_set);
	maskAllBits(BOHRER_FESTHALTEN, type_rem);

	// bohrer hochfahren
	do {
		// im intervall prüfen ob er schon ganz oben ist
		rt_sleep(100 * nano2count(1000000));
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
	} while ((val & BOHRER_OBEN) != BOHRER_OBEN);

	// bohrer hochfahren stoppen
	maskAllBits(BOHRER_HOCH, type_rem);

	// ausgang zurücksetzen
	maskAllBits(AUSWERFER_AUSGANG, type_rem);

	for(i = 0; i < 5; i++) {
		rt_mbx_send(&mbox[ausgang_mbox], &cmd, sizeof(cmd));
		rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));
		rt_mbx_send(&mbox[drehteller_mbox], &cmd, sizeof(cmd));
		rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));
	}
	rt_mbx_send(&mbox[ausgang_mbox], &cmd, sizeof(cmd));
	rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));
}

static void control(long x) {
	int k;
	// zähler für alle gesendeten Nachrichten (mbx)
	int messages_send = 0;
	// einlese wert für die mbx
	int cmd = 0;
	// einlese wert für DIGITAL_IN modbus
	short mod_value = 0;

	rt_printk("control: task started\n");

	/* Verbindung zum Modbus-Knoten herstellen */
	if ((fd_node = rt_modbus_connect("modbus-node")) == -1) {
		rt_printk("control: modbus_connect failed\n");
		return;
	}

	rt_printk("control: MODBUS communication opened\n");

	// tasks starten
	rt_task_resume(&task_drehteller);
	rt_task_resume(&task_ausgang);
	rt_task_resume(&task_pruefer);
	rt_task_resume(&task_bohrer);

	// programm initialisieren
	initProgram();


	while (1) {
		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// wenn bit 0 gesetzt ist, wurde ein Objekt erkannt
		if (((mod_value & IN_DREHTELLER_SENSOR) == IN_DREHTELLER_SENSOR)
				|| ((mod_value & IN_PRUEFER) == IN_PRUEFER)
				|| ((mod_value & IN_BOHRER) == IN_BOHRER)) {

			if ((mod_value & IN_DREHTELLER_SENSOR) == IN_DREHTELLER_SENSOR) {
				teile_pos[0] = 1;
			}

			// senden / empfangen sequenziell
			rt_mbx_send(&mbox[drehteller_mbox], &cmd, sizeof(cmd));
			rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));

			rt_printk("control message received\n");

			// teile shiften
			teile_pos[3] = teile_pos[2];
			teile_pos[2] = teile_pos[1];
			teile_pos[1] = teile_pos[0];
			teile_pos[0] = 0;
		}

		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// prüfer sensor überprüfen
		if ((mod_value & IN_PRUEFER) == IN_PRUEFER) {
			cmd = 0;
			rt_mbx_send(&mbox[pruefer_mbox], &cmd, sizeof(cmd));
			messages_send++;
		}

		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// bohrer überprüfen
		if ((mod_value & IN_BOHRER) == IN_BOHRER) {
			cmd = 0;

			// wenn das teil richtig liegt, wird gebohrt
			if (teile_pos[2] == 1) {
				rt_mbx_send(&mbox[bohrer_mbox], &cmd, sizeof(cmd));
				messages_send++;
			}
		}

		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// teil auswerfen
		if (teile_pos[3] > 0) {
			rt_mbx_send(&mbox[ausgang_mbox], &cmd, sizeof(cmd));
			messages_send++;
			// nach dem auswerfen wieder auf 0 setzen
			teile_pos[3] = 0;
		}

		// nachrichten empfangen
		for (k = 0; k < messages_send; k++) {
			rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));

			if (cmd == mbox_pruefer_wrong) {
				// 2 = "falsches" teil (also falsch herum)
				teile_pos[1] = 2;
			}
		}


		messages_send = 0;
		rt_sleep(100 * nano2count(1000000));
	}

	rt_modbus_disconnect(fd_node);
	rt_printk("control: MODBUS communication closed\n");
	rt_printk("control: task exited\n");
}

static void pruefer(long x) {
	int mbox_val = 0;
	short val;

	rt_printk("pruefer: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[pruefer_mbox], &mbox_val, sizeof(mbox_val));
		rt_printk("pruefer: message received %d\n", mbox_val);

		maskAllBits(PRUEFER_AN, type_set);

		// Prüfzeit geben
		rt_sleep(150 * nano2count(1000000));

		rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);

		// überprüfen ob das Teil richtig herum liegt
		if ((val & PRUEFER_NORMALLAGE) == PRUEFER_NORMALLAGE) {
			mbox_val = mbox_pruefer_right;
		} else {
			mbox_val = mbox_pruefer_wrong;
		}

		maskAllBits(PRUEFER_AN, type_rem);

		rt_printk("pruefer: send mbox %d\n", mbox_val);
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));
	}

	rt_printk("pruefer: task exited\n");
	return;
}

static void bohrer(long x) {
	int mbox_val = 0;
	short val;

	rt_printk("bohrer: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[bohrer_mbox], &mbox_val, sizeof(mbox_val));
		rt_printk("bohrer: message received\n");

		/* bohrer festhalten einschalten
		   bohrer runter fahren starten
		   bohrer anschalten
		*/
		maskAllBits((BOHRER_FESTHALTEN | BOHRER_RUNTER | BOHRER_AN), type_set);

		// bohrer runterfahren
		do {
			// im intervall prüfen ob er schon ganz unten ist
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
		} while ((val & BOHRER_UNTEN) != BOHRER_UNTEN);

		// zeit geben zum bohren
		rt_sleep(250 * nano2count(1000000));

		// alles zurück
		maskAllBits((BOHRER_FESTHALTEN | BOHRER_RUNTER | BOHRER_AN), type_rem);
		maskAllBits(BOHRER_HOCH, type_set);

		// bohrer hochfahren
		do {
			// im intervall prüfen ob er schon ganz oben ist
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
		} while ((val & BOHRER_OBEN) != BOHRER_OBEN);

		// bohrer hochfahren stoppen
		maskAllBits(BOHRER_HOCH, type_rem);

		mbox_val = mbox_bohrer;
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));
	}

	rt_printk("bohrer: task exited\n");
	return;
}

static void ausgang(long x) {
	int mbox_val = 0;

	rt_printk("ausgang: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[ausgang_mbox], &mbox_val, sizeof(mbox_val));

		maskAllBits(AUSWERFER_AUSGANG, type_set);

		// zeit zum auswerfen geben, 500ms für die schweren Teile
		rt_sleep(500 * nano2count(1000000));

		maskAllBits(AUSWERFER_AUSGANG, type_rem);

		mbox_val = mbox_ausgang;
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));
	}

	rt_printk("ausgang: task exited\n");
	return;
}

static void drehteller(long x) {
	short val = 0;
	int mbox_val = 0;

	rt_printk("drehteller: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[drehteller_mbox], &mbox_val, sizeof(mbox_val));
		rt_printk("drehteller: message received %d\n", mbox_val);

		maskAllBits(DREHTELLER_AN, type_set);

		// teller drehen solange bis der teller nicht mehr in position ist
		do {
			// im Intervall überprüfen ob der Drehteller in der richtigen Position ist
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);

		} while ((val & DREHTELLER_IN_POS) == DREHTELLER_IN_POS);

		maskAllBits(DREHTELLER_AN, type_rem);

		do {
			// im Intervall überprüfen ob der Drehteller nicht in der richtigen Position ist
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
		} while ((val & DREHTELLER_IN_POS) != DREHTELLER_IN_POS);

		// zeit geben sonst wird der sensor aktiviert
		rt_sleep(100 * nano2count(1000000));

		mbox_val = mbox_drehteller_in_pos;
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));
	}

	rt_printk("drehteller: task exited\n");
	return;
}

static void __exit
my_exit(void) {
	int i;
	stop_rt_timer();

	// semaphore löschen
	rt_sem_delete(&sem);

	// tasks löschen, entgegengesetzt
	rt_task_delete(&task_ausgang);
	rt_task_delete(&task_bohrer);
	rt_task_delete(&task_pruefer);
	rt_task_delete(&task_drehteller);
	rt_task_delete(&task_control);

	// mbx löschen
	for (i = 0; i < MBX_COUNT; i++) {
		rt_mbx_delete(&mbox[i]);
	}

	printk("processes unloaded\n");
}

static int __init
my_init(void) {
	int i;
	/* variables Timing basierend auf dem CPU-Takt */
	rt_set_oneshot_mode();
	/* Timer starten */
	start_rt_timer(0);
	/* Modbuskommunikation initialisieren */
	modbus_init();

	//Semaphor initialisieren
	rt_typed_sem_init(&sem, 1, CNT_SEM);

	// mbx initialisieren
	for (i = 0; i < MBX_COUNT; i++)
		rt_mbx_init(&mbox[i], sizeof(int));

	/* Taskinitialisierung
	 *
	 * &task = Adresse des zu initialisierenden TCB
	 * control = zum Task gehörende Funktion
	 * 0 = Uebergabeparameter an die zum Task gehoerende Funktion (long)
	 * 1024 = Stacksize
	 * 0 = Priorität des Tasks (0 = groesste)
	 * 0 = uses_fpu (Fliesskommaeinheit nicht benutzen); im Praktikum sollte es 0 bleiben
	 * NULL = &signal_handler; Referenz zu einer Fkt., die bei jeder Aktivierung
	 * des Tasks aufgerufen wird, ansonsten NULL
	 *
	 * Achtung: Nach der Initialisierung ist der Task "suspended"!
	 *
	 */
	if (rt_task_init(&task_control, control, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize control task\n");
		goto fail;
	}

	if (rt_task_init(&task_drehteller, drehteller, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize drehteller task\n");
		goto fail1;
	}

	if (rt_task_init(&task_pruefer, pruefer, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize pruefer task\n");
		goto fail2;
	}

	if (rt_task_init(&task_bohrer, bohrer, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize bohrer task\n");
		goto fail3;
	}

	if (rt_task_init(&task_ausgang, ausgang, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize ausgang task\n");
		goto fail4;
	}

	rt_task_resume(&task_control);
	printk("control task loaded\n");
	return (0);

	// tasks löschen, entgegengesetzt

	fail4:
	rt_task_delete(&task_bohrer);
	fail3:
	rt_task_delete(&task_pruefer);
	fail2:
	rt_task_delete(&task_drehteller);
	fail1:
	rt_task_delete(&task_control);

	fail:
	// semaphore löschen
	rt_sem_delete(&sem);

	// mbx löschen
	for (i = 0; i < MBX_COUNT; i++) {
		rt_mbx_delete(&mbox[i]);
	}

	stop_rt_timer();
	return (1);
}

module_exit(my_exit)
module_init(my_init)
