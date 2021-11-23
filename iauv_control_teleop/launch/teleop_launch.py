from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()
            
    sl.declare_arg('namespace', default_value='bluerov2')
    
    with sl.group(ns=sl.arg('namespace')):
        
        sl.node('joy','joy_node')
        
        # TODO spawn the teleop node once it works

    return sl.launch_description()
