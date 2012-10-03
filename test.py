import netv

fpga = netv.fpga()

#fpga.dump_registers()

print "Device ID: %s" % fpga.deviceId()
print "FPGA reports a version code of: %d" % fpga.version()

