sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432


# One of the follwing commands seemed to solve this error:
#    Error: EnvironmentError: IOError: Radio ctrl (A) packet parse error - AssertionError: packet_info.packet_count == (seq_to_ack & 0xfff)
#  (happening while uhd_usrp_probe with MTU 9000)
sudo sysctl -w net.core.rmem_default=33554432
sudo sysctl -w net.core.wmem_default=33554432
sudo sysctl -w net.core.optmem_max=5242870
sudo sysctl -w net.core.netdev_max_backlog=300000
sudo sysctl -w net.core.netdev_budget=600



# disable cpu throttling
sudo cpufreq-set -g PERFORMANCE

# disable interrupt coallesing:
sudo ethtool -C ens85f0 adaptive-tx off
sudo ethtool -C ens85f1 adaptive-tx off
sudo ethtool -C ens85f2 adaptive-tx off
sudo ethtool -C ens85f3 adaptive-tx off

sudo ethtool -C ens85f0 adaptive-rx off
sudo ethtool -C ens85f1 adaptive-rx off
sudo ethtool -C ens85f2 adaptive-rx off
sudo ethtool -C ens85f3 adaptive-rx off

sudo ethtool -C ens93f0 adaptive-tx off
sudo ethtool -C ens93f1 adaptive-tx off
sudo ethtool -C ens93f2 adaptive-tx off
sudo ethtool -C ens93f3 adaptive-tx off

sudo ethtool -C ens93f0 adaptive-rx off
sudo ethtool -C ens93f1 adaptive-rx off
sudo ethtool -C ens93f2 adaptive-rx off
sudo ethtool -C ens93f3 adaptive-rx off

sudo ethtool -C ens118f0 adaptive-tx off
sudo ethtool -C ens118f1 adaptive-tx off
sudo ethtool -C ens118f2 adaptive-tx off
sudo ethtool -C ens118f3 adaptive-tx off

sudo ethtool -C ens118f0 adaptive-rx off
sudo ethtool -C ens118f1 adaptive-rx off
sudo ethtool -C ens118f2 adaptive-rx off
sudo ethtool -C ens118f3 adaptive-rx off

sudo /home/radar/repos/SuperDARN_UHD_Server/tools/change_priority_by_name.sh ksoftirqd 60



