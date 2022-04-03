project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')
eigen_dirs = include_directories('/usr/local/include/eigen3')
eigen_deps = declare_dependency(
    include_directories : eigen_dirs,
    link_args : []
)

all_deps = [liblcm, eigen_deps]

src = ['main.cpp', 'stateMachine.cpp', 'rover.cpp', 'pid.cpp', 'utilities.cpp',
			'search/spiralInSearch.cpp', 'search/lawnMowerSearch.cpp', 'search/searchStateMachine.cpp', 'search/spiralOutSearch.cpp',
            'gate_search/gateStateMachine.cpp', 'gate_search/diamondGateSearch.cpp', 'obstacle_avoidance/obstacleAvoidance.cpp']

executable('jetson_nav',
    sources: src,
    dependencies : all_deps,
    install : true)
