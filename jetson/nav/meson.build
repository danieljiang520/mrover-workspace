project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')
all_deps = [liblcm]

src = ['main.cpp', 'stateMachine.cpp', 'rover.cpp', 'pid.cpp', 'utilities.cpp',
			'search/spiralInSearch.cpp', 'search/lawnMowerSearch.cpp', 'search/searchStateMachine.cpp', 'search/spiralOutSearch.cpp',
            'gate_search/gateStateMachine.cpp', 'gate_search/diamondGateSearch.cpp', 'obstacle_avoidance/obstacleAvoidance.cpp'
            , 'obstacle_avoidance/test_getBearingDecision.cpp']

executable('jetson_nav',
    sources: src,
    dependencies : all_deps,
    install : true)
