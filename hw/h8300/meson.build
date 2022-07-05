h8300_ss = ss.source_set()
h8300_ss.add(when: 'CONFIG_H8300_KANEBEBE', if_true: files('kanebebe.c'))
h8300_ss.add(when: 'CONFIG_H8300_EDOSK2674', if_true: files('edosk2674.c'))
h8300_ss.add(when: 'CONFIG_H83069', if_true: files('h83069.c'))
h8300_ss.add(when: 'CONFIG_H8S2674', if_true: files('h8s2674.c'))

hw_arch += {'h8300': h8300_ss}
