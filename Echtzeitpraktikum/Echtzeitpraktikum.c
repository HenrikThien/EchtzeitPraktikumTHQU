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

//aktoren
#define BOHRER_AN (1 << 0)
#define DREHTELLER_AN (1 << 1)
#define BOHRER_RUNTER (1 << 2)
#define BOHRER_HOCH (1 << 3)
#define BOHRER_FESTHALTEN (1 << 4)
#define PRUEFER_AN (1 << 5)
#define AUSWERFER_AUSGANG (1 << 6)
#define AUSWERFER_EINGANG (1 << 7)

//aktoren aus
#define BOHRER_AUS (0 << 0)
#define DREHTELLER_AUS (0 << 1)
#define BOHRER_RUNTER_AUS (0 << 2)
#define BOHRER_HOCH_AUS (0 << 3)
#define BOHRER_FESTHALTEN_AUS (0 << 4)
#define PRUEFER_AUS (0 << 5)
#define AUSWERFER_AUSGANG_AUS (0 << 6)

static RT_TASK task_control;
static RT_TASK task_drehteller;
static RT_TASK task_pruefer;
static RT_TASK task_bohrer;
static RT_TASK task_ausgang;

static int fd_node;
static SEM sem;

static int MBX_COUNT = 5;
static MBX mbox[5];

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

static void control(long x) {
	int k;
	int messages_send = 0;
	int cmd = 0;
	short mod_value = 0;
	int teile_pos[4];
	int temp[4];
	int auswerfen = 0;

	rt_printk("control: task started\n");

	/* Verbindung zum Modbus-Knoten herstellen */
	if ((fd_node = rt_modbus_connect("modbus-node")) == -1) {
		rt_printk("control: modbus_connect failed\n");
		return;
	}

	rt_printk("control: MODBUS communication opened\n");

	rt_task_resume(&task_drehteller);
	rt_task_resume(&task_pruefer);
	rt_task_resume(&task_bohrer);
	//rt_task_resume(&task_ausgang);

	while (1) {
		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// wenn bit 0 gesetzt ist, wurde ein Objekt erkannt
		if (((mod_value & IN_DREHTELLER_SENSOR) == IN_DREHTELLER_SENSOR)
				|| ((mod_value & IN_PRUEFER) == IN_PRUEFER)
				|| ((mod_value & IN_BOHRER) == IN_BOHRER)) {
			rt_printk("mod_value: %d\n", mod_value);

			if ((mod_value & IN_DREHTELLER_SENSOR) == IN_DREHTELLER_SENSOR) {
				teile_pos[0] = 1;
			}

			rt_mbx_send(&mbox[drehteller_mbox], &cmd, sizeof(cmd));
			rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));

			if (cmd == mbox_drehteller_in_pos) {
				//shifte das array
				//memmove(teile_pos, teile_pos + 1, sizeof teile_pos - sizeof *teile_pos);
				int i;
				for (i = 0; i < 4; i++) {
					temp[i] = teile_pos[i];
				}
				for (i = 0; i < 3; i++) {
					teile_pos[0] = 0;
					teile_pos[i + 1] = temp[i];
				}
			}
		}

		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		if ((mod_value & IN_PRUEFER) == IN_PRUEFER) {
			cmd = 0;
			rt_mbx_send(&mbox[pruefer_mbox], &cmd, sizeof(cmd));
			messages_send++;
		}

		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		if ((mod_value & IN_BOHRER) == IN_BOHRER) {
			cmd = 0;
			if (teile_pos[2] == 1) {
				rt_mbx_send(&mbox[bohrer_mbox], &cmd, sizeof(cmd));
				messages_send++;
			}
		}

		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// teil auswerfen
		if (auswerfen == 1) {
			//rt_mbx_send(&mbox[ausgang_mbox], &cmd, sizeof(cmd));
			//messages_send++;
		}

		for (k = 0; k < messages_send; k++) {
			rt_mbx_receive(&mbox[control_mbox], &cmd, sizeof(cmd));

			if (cmd == mbox_bohrer) {
				auswerfen = 1;
			}

			if (cmd == mbox_pruefer_wrong) {
				// 2 = "falsches" teil (also falsch herum)
				teile_pos[1] = 2;
			}
		}

		messages_send = 0;
	}
}

static void pruefer(long x) {
	int mbox_val = 0;
	short val;

	rt_printk("pruefer: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[pruefer_mbox], &mbox_val, sizeof(mbox_val));
		rt_printk("pruefer: message received %d\n", mbox_val);

		rt_sem_wait(&sem);
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, PRUEFER_AN);
		rt_sem_signal(&sem);

		rt_sleep(100 * nano2count(1000000));
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);

		if ((val & PRUEFER_NORMALLAGE) == PRUEFER_NORMALLAGE) {
			mbox_val = mbox_pruefer_right;
		} else {
			mbox_val = mbox_pruefer_wrong;
		}

		rt_sem_wait(&sem);
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, PRUEFER_AUS);
		rt_sem_signal(&sem);

		rt_printk("pruefer: send mbox %d\n", mbox_val);
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));

	}

	rt_printk("pruefer: task exited\n");

}

static void bohrer(long x) {
	int mbox_val = 0;
	short val;

	rt_printk("bohrer: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[bohrer_mbox], &mbox_val, sizeof(mbox_val));
		rt_printk("bohrer: message received\n");

		rt_sem_wait(&sem);
		rt_modbus_set(fd_node, DIGITAL_OUT, 0,
				(BOHRER_FESTHALTEN | BOHRER_RUNTER | BOHRER_AN));
		rt_sem_signal(&sem);

		// bohrer runterfahren
		do {
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
		} while ((val & BOHRER_UNTEN) != BOHRER_UNTEN);

		// zeit geben
		rt_sleep(250 * nano2count(1000000));

		// alles zurück
		rt_sem_wait(&sem);
		rt_modbus_set(fd_node, DIGITAL_OUT, 0,
				(BOHRER_FESTHALTEN_AUS | BOHRER_HOCH | BOHRER_AUS));
		rt_sem_signal(&sem);

		// bohrer hochfahren
		do {
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
		} while ((val & BOHRER_OBEN) != BOHRER_OBEN);

		rt_printk("bohrer: send mbox\n");
		mbox_val = mbox_bohrer;
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));
	}

	rt_printk("bohrer: task exited\n");
}

static void ausgang(long x) {
	int mbox_val = 0;

	rt_printk("ausgang: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));

		rt_sem_wait(&sem);
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, AUSWERFER_AUSGANG);
		rt_sem_signal(&sem);

		rt_sleep(250 * nano2count(1000000));

		rt_sem_wait(&sem);
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, AUSWERFER_AUSGANG_AUS);
		rt_sem_signal(&sem);

		mbox_val = mbox_ausgang;
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));
	}

	rt_printk("ausgang: task exited\n");
}

static void drehteller(long x) {
	short val = 0;
	int mbox_val = 0;

	rt_printk("drehteller: task started\n");

	while (1) {
		rt_mbx_receive(&mbox[drehteller_mbox], &mbox_val, sizeof(mbox_val));
		rt_printk("drehteller: message received %d\n", mbox_val);

		// bitmaske an modbus senden
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, DREHTELLER_AN);

		// teller drehen solange bis der teller nicht mehr in position ist
		do {
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);

		} while ((val & DREHTELLER_IN_POS) == DREHTELLER_IN_POS);

		// 0 an modbus senden und stoppen
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, (0 << 1));

		do {
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);
		} while ((val & DREHTELLER_IN_POS) != DREHTELLER_IN_POS);

		rt_sleep(100 * nano2count(1000000));

		rt_modbus_set(fd_node, DIGITAL_OUT, 0, DREHTELLER_AUS);

		mbox_val = mbox_drehteller_in_pos;
		rt_printk("drehteller: send mbox\n");
		rt_mbx_send(&mbox[control_mbox], &mbox_val, sizeof(mbox_val));

		rt_sem_signal(&sem);
	}

	rt_printk("drehteller: task exited\n");
}

static void __exit
my_exit(void) {
	int i;
	stop_rt_timer();

	rt_sem_delete(&sem);

	rt_task_delete(&task_ausgang);
	rt_task_delete(&task_bohrer);
	rt_task_delete(&task_pruefer);
	rt_task_delete(&task_drehteller);
	rt_task_delete(&task_control);

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
		goto fail;
	}

	if (rt_task_init(&task_pruefer, pruefer, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize pruefer task\n");
		goto fail;
	}

	if (rt_task_init(&task_bohrer, bohrer, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize bohrer task\n");
		goto fail;
	}

	if (rt_task_init(&task_ausgang, ausgang, 0, STACKSIZE, 0, 0, NULL)) {
		printk("cannot initialize ausgang task\n");
		goto fail;
	}

	rt_task_resume(&task_control);
	printk("control task loaded\n");
	return (0);

	fail: stop_rt_timer();
	return (1);
}

module_exit(my_exit)
module_init(my_init)
