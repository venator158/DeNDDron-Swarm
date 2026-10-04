Simulation-only TLS material for the radio's QUIC link (control plane): a throwaway CA and one
certificate/key shared by every node. Not a secret and not for hardware: there, each node gets its
own key (see "No security" in the top-level README's known limitations).
