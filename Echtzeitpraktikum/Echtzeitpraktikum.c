#include <rtai_mbx.h>
#include <rtai_sched.h>

#include <uint128.h>
#include <sys/rtai_modbus.h>

#define STACKSIZE 10000

MODULE_LICENSE("GPL");

#define IN_POS	(1 << 5)

static RT_TASK task_control;
static RT_TASK task_drehteller;
//static RT_TASK task_pruefer;
//static RT_TASK task_bohrer;
//static RT_TASK task_ausgang;

static int fd_node;

static void control(long x) {
	rt_printk("control: task started\n");

	/* Verbindung zum Modbus-Knoten herstellen */
	if ((fd_node = rt_modbus_connect("modbus-node")) == -1) {
		rt_printk("control: modbus_connect failed\n");
		return;
	}

	rt_printk("control: MODBUS communication opened\n");

	rt_task_resume(&task_drehteller);

	rt_printk("control: task exited\n");
}

static void drehteller(long x) {
	short val = 0;
	short set_bitmask = 0b00000010;

	rt_printk("drehteller: task started\n");

	while (1) {
		if (rt_modbus_get(fd_node, DIGITAL_IN, 0, (unsigned short *) &val))
			goto fail;

		if (val & (1 << 0)) {
			rt_printk("wert ist 1\n");
			// teller drehen solange bis der teller nicht mehr in position ist
			if (rt_modbus_set(fd_node, DIGITAL_OUT, 0, set_bitmask))
				goto fail;
			do {
				rt_sleep(100 * nano2count(1000000));
				if (rt_modbus_get(fd_node, DIGITAL_IN, 0, (unsigned short *) &val))
					goto fail;

			} while((val & IN_POS) == IN_POS);

			if (rt_modbus_set(fd_node, DIGITAL_OUT, 0, 0))
				goto fail;

			do {
				rt_sleep(100 * nano2count(1000000));
				if (rt_modbus_get(fd_node, DIGITAL_IN, 0, (unsigned short *) &val))
					goto fail;

			} while((val & IN_POS) != IN_POS);

			rt_sleep(100 * nano2count(1000000));

		}

		else {
			rt_printk("wert ist 0\n");
			// teller drehen
			if (rt_modbus_set(fd_node, DIGITAL_OUT, 0, 0))
				goto fail;
		}
	}

	fail:
		rt_modbus_disconnect(fd_node);
		rt_printk("control: MODBUS disconnected\n");
		rt_printk("drehteller: task exited\n");
}
static void __exit
my_exit(void) {
	stop_rt_timer();
	rt_task_delete(&task_drehteller);
	rt_task_delete(&task_control);

	printk("rtai_example unloaded\n");

}

static int __init
my_init(void) {
	/* variables Timing basierend auf dem CPU-Takt */
	rt_set_oneshot_mode();
	/* Timer starten */
	start_rt_timer(0);
	/* Modbuskommunikation initialisieren */
	modbus_init();

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

	rt_task_resume(&task_control);
	printk("rtai_example loaded\n");
	return (0);

	fail: stop_rt_timer();
	return (1);
}

module_exit(my_exit)
module_init(my_init)
