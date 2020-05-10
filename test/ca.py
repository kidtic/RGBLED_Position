import math

alp=math.atan2(19.95,157.9)
theta=math.atan2(58.59,147.98)
r=math.sqrt(157.9**2+19.95**2)
x=r*math.cos(17*alp)
y=r*math.sin(17*alp)
print(x,y)
