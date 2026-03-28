with open('/home/venator/denddron/DeNDDron-Swarm/sim/GazeboSimulator.cpp', 'r') as f:
    text = f.read()

text = text.replace('auto cl1 =', 'z_owned_closure_sample_t cl1 =')
text = text.replace('auto cl2 =', 'z_owned_closure_sample_t cl2 =')

import re
text = re.sub(r'z_close\(_session\);', r'z_close(z_move(_session));', text)
text = re.sub(r'_session\s*=\s*nullptr;', r'// _session = nullptr;', text)
text = re.sub(r'if\s*\(\w*\+?_session\)', r'if(z_check(_session))', text)

# despawn drone
text = text.replace('z_declare_publisher(\n        _session, ', 'z_declare_publisher(\n        z_loan(_session), ')

# Remove the initial _session(nullptr) in the constructor
text = re.sub(r'_session\s*\(\s*nullptr\s*\)\s*,?', '', text)

with open('/home/venator/denddron/DeNDDron-Swarm/sim/GazeboSimulator.cpp', 'w') as f:
    f.write(text)
