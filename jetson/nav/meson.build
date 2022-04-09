project('nav', 'cpp', default_options : ['cpp_std=gnu++1z'])

liblcm = dependency('lcm')
all_deps = [liblcm]

executable('jetson_nav', 'main.cpp', 'stateMachine.cpp', 'rover.cpp', 'pid.cpp', 'utilities.cpp',
			'search/searchFromPathFile.cpp', 'search/searchStateMachine.cpp',
            'gate_search/gateStateMachine.cpp', 'gate_search/diamondGateSearch.cpp', 'courseProgress.cpp', 'environment.cpp', 'obstacle_avoidance/obstacleAvoidance.cpp',
            'obstacle_avoidance/test_getIdealDesiredBearing.cpp', 'obstacle_avoidance/test_getBearingDecision.cpp',
           dependencies : [liblcm],
           install : true)
