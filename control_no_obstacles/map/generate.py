f = open('45.txt','w')

x = -15.00  
y = 41.70  
yaw = 4.71
while y > -20:
    y -=1
    f.write(str(x)+'\t'+str(y)+'\t'+str(yaw) + '\n')

f.close()
