SNS_VIP=$2
SNS_VIP_INDEX=$3
#/etc/rc.d/init.d/functions
case "$1" in
start)
       ifconfig lo:${SNS_VIP_INDEX:-0} $SNS_VIP netmask 255.255.255.255 broadcast $SNS_VIP
       /sbin/route add -host $SNS_VIP dev lo
       #echo "1" >/proc/sys/net/ipv4/conf/lo/arp_ignore
       #echo "2" >/proc/sys/net/ipv4/conf/lo/arp_announce
       #echo "1" >/proc/sys/net/ipv4/conf/all/arp_ignore
       #echo "2" >/proc/sys/net/ipv4/conf/all/arp_announce
       # sysctl -p >/dev/null 2>&1
       echo "RealServer Start OK"
       ;;
stop)
       ifconfig lo:${SNS_VIP_INDEX:-0} down
       route del $SNS_VIP >/dev/null 2>&1
       #echo "0" >/proc/sys/net/ipv4/conf/lo/arp_ignore
       #echo "0" >/proc/sys/net/ipv4/conf/lo/arp_announce
       #echo "0" >/proc/sys/net/ipv4/conf/all/arp_ignore
       #echo "0" >/proc/sys/net/ipv4/conf/all/arp_announce
       echo "RealServer Stoped"
       ;;
*)
       echo "Usage: $0 {start|stop}"
       exit 1
esac
exit 0