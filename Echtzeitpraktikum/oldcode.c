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

static MBX mbox[5];

// mbx->tasks messages
#define control_mbx 0
#define drehteller_mbx 1
#define pruefer_mbx 2
#define bohrer_mbx 3
#define ausgang_mbx 4

// tasks->mbx messages
#define mbx_drehteller_in_pos 0
#define mbx_pruefer 1
#define mbx_bohrer 2
#define mbx_ausgang 3


static void control(long x) {
	short mod_value = 0;
	int cmd = 1;
	int ret;

	rt_printk("control: task started\n");

	/* Verbindung zum Modbus-Knoten herstellen */
	if ((fd_node = rt_modbus_connect("modbus-node")) == -1) {
		rt_printk("control: modbus_connect failed\n");
		return;
	}

	rt_printk("control: MODBUS communication opened\n");

	rt_task_resume(&task_drehteller);

	while (1) {
		// einlesen der Werte vom Sensor
		rt_modbus_get(fd_node, DIGITAL_IN, 0, &mod_value);

		// wenn bit 0 gesetzt ist, wurde ein Objekt erkannt
		if (mod_value & IN_DREHTELLER_SENSOR) {
			rt_printk("es liegt etwas drauf\n");
			rt_mbx_send(mbox, &cmd, sizeof(int));
		} else {
			//wenn das 0 bit nicht gesetzt ist, nicht drehen (Kein Objekt erkannt)
			rt_modbus_set(fd_node, DIGITAL_OUT, 0, 0);
		}

		rt_sleep(200 * nano2count(1000000));
	}

	fail: rt_modbus_disconnect(fd_node);
	rt_printk("control: MODBUS disconnected\n");
	rt_printk("control: task exited\n");
}

static void pruefer(long x) {
	rt_printk("pruefer: task started\n");

	rt_printk("pruefer: task exited\n");
	return;
}

static void bohrer(long x) {
	rt_printk("bohrer: task started\n");

	rt_printk("bohrer: task exited\n");
	return;
}

static void ausgang(long x) {
	rt_printk("ausgang: task started\n");

	rt_printk("ausgang: task exited\n");
	return;
}

static void drehteller(long x) {
	short val = 0;
	rt_printk("drehteller: task started\n");

	rt_sem_wait(&sem);

	while (1) {
		while (1) {
			int ret;
			rt_mbx_receive(mbox, &ret, sizeof(int));

			if (ret == 1) {
				rt_printk("drehteller: mbox empfangen\n");
				break;
			}
		}

		// bitmaske an modbus senden
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, DREHTELLER_AN);

		// teller drehen solange bis der teller nicht mehr in position ist
		do {
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);

		} while ((val & DREHTELLER_IN_POS) == DREHTELLER_IN_POS);

		// 0 an modbus senden und stoppen
		rt_modbus_set(fd_node, DIGITAL_OUT, 0, 0);

		do {
			rt_sleep(100 * nano2count(1000000));
			rt_modbus_get(fd_node, DIGITAL_IN, 0, &val);

		} while ((val & DREHTELLER_IN_POS) != DREHTELLER_IN_POS);

		rt_sleep(100 * nano2count(1000000));
	}

	rt_sem_signal(&sem);

	rt_printk("drehteller: task exited\n");
	return;
}

static void __exit
my_exit(void) {
	stop_rt_timer();
	rt_sem_delete(&sem);
	rt_task_delete(&task_ausgang);
	rt_task_delete(&task_bohrer);
	rt_task_delete(&task_pruefer);
	rt_task_delete(&task_drehteller);
	rt_task_delete(&task_control);

	int i;
	for (i = 0; i < 4; i++) {
		rt_mbx_delete(&mbox[i]);
	}

	rt_sem_delete(&sem);
	printk("processes unloaded\n");
}

static int __init
my_init(void) {
	/* variables Timing basierend auf dem CPU-Takt */
	rt_set_oneshot_mode();
	/* Timer starten */
	start_rt_timer(0);
	/* Modbuskommunikation initialisieren */
	modbus_init();

	//Semaphor initialisieren
	rt_typed_sem_init(&sem, 1, CNT_SEM);
	rt_mbx_init(mbox, sizeof(int));

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
