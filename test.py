import netv

fpga = netv.fpga()

print "Device ID: %s" % fpga.deviceId()
print "FPGA reports a version code of: %d" % fpga.version()


print "Before :", fpga.alphaState
fpga.alphaState = True
print "After :", fpga.alphaState
fpga.alphaState = False
print "Final :", fpga.alphaState



print fpga.alphaValue
fpga.alphaValue = 0.1
print fpga.alphaValue
fpga.alphaValue = 0.2
print fpga.alphaValue
fpga.alphaValue = 0.3
print fpga.alphaValue
fpga.alphaValue = 0.4
print fpga.alphaValue
fpga.alphaValue = 0.5
print fpga.alphaValue
fpga.alphaValue = 0.6
print fpga.alphaValue
fpga.alphaValue = 0.7
print fpga.alphaValue
fpga.alphaValue = 0.8
print fpga.alphaValue
fpga.alphaValue = 0.9
print fpga.alphaValue
fpga.alphaValue = 1
print fpga.alphaValue

#print "after alpha value:", fpga.alphaValue
#fpga.dump_register(0xc)

#fpga.alphaOff()
#print fpga.alphaState()
#fpga.dump_registers()

#fpga.dump_registers()
#fpga.compositingOn()
#fpga.dump_registers()
