sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
sudo sysctl -w net.core.wmem_max=33554432

# disable interrupt coallesing:
#sudo ethtool -C enp9s0f0 adaptive-tx off

