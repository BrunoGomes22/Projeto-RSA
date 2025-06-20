package org.nap.fleetman.server.utils

import org.apache.commons.net.util.SubnetUtils
import org.apache.commons.net.util.SubnetUtils.SubnetInfo

class NetUtils {

    static List<String> getAddresses(String netAddress) {
        SubnetInfo subnet = (new SubnetUtils(netAddress)).getInfo()
        InetAddress addr = null
        for (def netInterface: NetworkInterface.getNetworkInterfaces()) {
            InetAddress address = netInterface.inetAddresses().find { address ->
                address instanceof Inet4Address && subnet.isInRange(address.hostAddress)
            }
            if (address != null) {
                addr = address
                break
            }
        }
        if (addr == null)
            return []
        def hardwareAddr = NetworkInterface.getByInetAddress(addr).getHardwareAddress()
        String[] hexadecimalFormat = new String[hardwareAddr.length]
        for (int i = 0; i < hardwareAddr.length; i++)
            hexadecimalFormat[i] = String.format("%02X", hardwareAddr[i]).toLowerCase()
        String macAddress = String.join(":", hexadecimalFormat)
        return [addr.hostAddress, macAddress]
    }
}
