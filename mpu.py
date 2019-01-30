#一节互补解算
import math
i1=0
x=0 
y=0
angleAx=0
angleAy=0
one_filter_angle=[0,0,0,0,0,0,0,0]
def one_filter(ax,ay,az,gx,gy,gz):
    global i1
    global angleAy
    global angleAx
    global x
    global y
    K1 =0.3; # 对加速度计取值的权重
    global one_filter_angle
    dt=0.008;#注意：dt的取值为滤波器采样时间
    gyo=gy/65.5
    gzo=gz/65.5
    gxo=gx/65.5
    if abs(gyo)<0.07:
        gyo=0
    if abs(gxo)<0.07:
        gxo=0
    if ay!=0 or az!=0:
        angleAx=math.atan(ax/math.sqrt(ay*ay+az*az))*180/math.pi;#计算与x轴夹角
    if ax!=0 or az!=0:
        angleAy=math.atan(ay/math.sqrt(ax*ax+az*az))*180/math.pi
    if i1==0:
        one_filter_angle[0]=angleAx
        one_filter_angle[1]=angleAy
        x=angleAx
        y=angleAy
        print('dd',x,y,gyo*dt)
    i1=i1+1
    if i1==10:
         i1=1
    one_filter_angle[0] = K1 * angleAx+ (1-K1) * (one_filter_angle[0] - gyo* dt);
    one_filter_angle[1] = K1 * angleAy+ (1-K1) * (one_filter_angle[1] + gxo* dt);
    x=x-gyo*dt
    y=y+gxo*dt
    one_filter_angle[2]=angleAx
    one_filter_angle[3]=angleAy
    one_filter_angle[4]=x
    one_filter_angle[5]=y
    one_filter_angle[6]=gyo
    one_filter_angle[7]=gxo
    #print(one_filter_angle)
    return one_filter_angle;
