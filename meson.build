project('lx2160-sdx', 'c',
  version : '0.1.0',
  license : 'BSD-3-Clause',
  meson_version : '>= 0.56',
  default_options : [
    'c_std=c11',
    'warning_level=3',
    'buildtype=release',
  ],
)

add_project_arguments(
  '-Wshadow',
  '-Wstrict-prototypes',
  language : 'c',
)

link_args = []
if get_option('static')
  link_args += ['-static', '-static-libgcc']
endif

executable('lx2160-sdx',
  'src/lx2160-sdx.c',
  link_args : link_args,
  install : true,
)

executable('lx2160-pcie',
  'src/lx2160-pcie.c',
  link_args : link_args,
  install : true,
)

executable('lx2160-eth',
  'src/lx2160-eth.c',
  link_args : link_args,
  install : true,
)

executable('lx2160-sata',
  'src/lx2160-sata.c',
  link_args : link_args,
  install : true,
)

executable('lx2160-linkmon',
  'src/lx2160-linkmon.c',
  link_args : link_args,
  install : true,
)
