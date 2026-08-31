# Experiments

Profiles record delay, jitter, loss, reorder, duplicate, pose rate, buffer delay, and a random seed. On Linux, a `TcNetemStrategy` applies and restores `tc netem`; on Windows, `UdpProxyStrategy` is `network_proxy`. Record raw CSV/JSON, graphs, p50/p95/p99 latency, position/rotation error, underruns, and limitations. Use same-machine independent processes for ground-truth latency experiments.
