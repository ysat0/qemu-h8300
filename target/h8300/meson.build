gen = [
  decodetree.process('insns.decode', extra_args: [ '--varinsnwidth', '32' ])
]

h8300_ss = ss.source_set()
h8300_ss.add(gen)
h8300_ss.add(files(
  'translate.c',
  'op_helper.c',
  'helper.c',
  'cpu.c',
  'gdbstub.c',
  'disas.c'))

target_arch += {'h8300': h8300_ss}
target_softmmu_arch += {'h8300': ss.source_set() }
