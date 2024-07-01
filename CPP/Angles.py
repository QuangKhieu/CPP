import math
import numpy as np
import matplotlib.pyplot as plt

def calculate_polygon_area(vertices):
    n = len(vertices)
    area = 0.0
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i+1) % n]  # Lấy đỉnh kế tiếp (hoặc đỉnh đầu tiên nếu i là đỉnh cuối cùng)
        area += x1*y2 - x2*y1
    return abs(area) / 2.0
vertices = [[59, -40],[-10,-40],[-20, -79], [-63, 24], [-31, 61], [52, 26]]
area = calculate_polygon_area(vertices)


def getAngle(knee, hip, shoulder):
    ang = math.degrees(math.atan2(shoulder[1]-hip[1], shoulder[0]-hip[0]) - math.atan2(knee[1]-hip[1], knee[0]-hip[0]))
    return ang + 360 if ang < 0 else ang

def takeA(K):
    for i in range(len(K)-1):
        if i==len(K) - 2:
            print(getAngle(K[i],K[i+1],K[1]))
        else:
            print(getAngle(K[i],K[i+1],K[i+2]))
def checkangle(K):
    point_angle = []
    for i in range(len(K)-1):
        if i == len(K)-2:
            if getAngle(K[i],K[i+1],K[1]) > 180:
                point_angle.append(i+1)
        else:
            if getAngle(K[i],K[i+1],K[i+2]) > 180:
                point_angle.append(i+1)

    return point_angle



#pt duong thang
def line_eq(point_1, point_2):
    A = point_1[0]
    B = point_1[1]
    C = point_2[0]
    D = point_2[1]

    a = B - D
    b = C - A
    c = a * -A + b * -B
    return a, b, c

# tim diem giao
def find_intersect_point(points, index_center):
    point1 = points[index_center]

    point2 = points[index_center-1]

    a1, b1, c1 = line_eq(point1, point2)
    tow  = index_center + 1
    if index_center == len(points) - 1:
        tow = 0
    point2 = points[tow]
    a2, b2, c2 = line_eq(point1, point2)
    no = []
    points.append(points[0])
    #print(a2, b2, c2)
    for i in range(len(points)-1):
        if(check_side([a1, b1, c1], points[i], points[i+1])):
            a, b, c = line_eq(points[i], points[i+1])
            x = eq2_solve([a,b,c], [a1, b1, c1])
            no.append(list(x))
        if(check_side([a2, b2, c2], points[i], points[i+1])):
            a, b, c = line_eq(points[i], points[i+1])
            x = eq2_solve([a,b,c], [a2, b2, c2])
            no.append(list(x))

    return no

#kiem tra vi tri tuong doi
def check_side(paraline, point_1, point_2):
    check1 = point_1[0] * paraline[0] + point_1[1] * paraline[1] + paraline[2]
    check2 = point_2[0] * paraline[0] + point_2[1] * paraline[1] + paraline[2]
    if check1*check2 < 0:
        return True
    else:
        return False
def eq2_solve(para_1, para_2):
    a1 = para_1[0]
    b1 = para_1[1]
    c1 = para_1[2]
    a2 = para_2[0]
    b2 = para_2[1]
    c2 = para_2[2]

    A = [[a1, b1], [a2, b2]]
    B = [-c1, -c2]

    x = np.linalg.solve(A, B)
    return x

def sort_points(sub_points):
    angle_vec = []
    new_sub_points = []
    new_sub_points.extend(sub_points)
    for i in range(len(sub_points)-1):
        #vector.append([sub_points[i+1][0] - sub_points[i][0], sub_points[i+1][1] - sub_points[i][1]])
        angle_vec.append( math.atan2(sub_points[i+1][1] - sub_points[0][1], sub_points[i+1][0] - sub_points[0][0]))
    if (max(angle_vec)>math.pi/2) and (min(angle_vec)<-math.pi/2):
        for i in range (len(angle_vec)):
            if angle_vec[i]<0:
                angle_vec[i] = angle_vec[i] + 2*math.pi
    for i in range(len(sub_points)-1):
        temp = angle_vec.index(max(angle_vec[i:]))
        angle_vec[i], angle_vec[temp] = angle_vec[temp], angle_vec[i]
        new_sub_points[i+1], new_sub_points[temp+1] = new_sub_points[temp+1], new_sub_points[i+1]
    return new_sub_points
def take_sub_Points(points, center, intersect_point): #tim toa do diem cua 2 phan map
    sub_point1 = [center, intersect_point]
    sub_point2 = [center, intersect_point]
    special_point = []
    flag_point = []
    # tim dinh da giac thuoc duong thang chua center, intersect_point
    index = points.index(center)
    a, b, c = line_eq(center, intersect_point)
    tow = index + 1
    if(index == len(points) -1 ):
        tow = 0

    check =  a*points[tow][0] + b*points[tow][1] + c
    if(check < 0.001):
        special_point = points[tow]
    else:
        special_point = points[index-1]
    index_special_point = points.index(special_point)
    new_points = []
    new_points.extend(points)
    new_points.remove(points[index_special_point])
    new_points.remove(points[index])

    #find flag_point : ke cua special point khac center
    tow2 = index_special_point+1
    if(index_special_point == len(points) -1 ):
        tow2 = 0
    if(points[tow2] != center ):
        flag_point = points[tow2]
    elif(points[index_special_point-1] != center):
        flag_point = points[index_special_point-1]

    # print('flag_point', flag_point)
    # print('special_point', special_point)
    # #tim sub_point chua sap xep
    #
    temp1 = a * new_points[0][0] + b * new_points[0][1] + c
    sub_point1.append(new_points[0])
    # print('pp',new_points)
    flag  = 1
    if new_points[0] == flag_point:
        sub_point1.append(special_point)
        flag = 1
    for i in range(1, len(new_points) - 1):
        temp2 = a * new_points[i][0] + b * new_points[i][1] + c
        if (temp1 * temp2 > 0):
            sub_point1.append(new_points[i])
            if new_points[i] == flag_point and i != len(new_points) - 1:
                sub_point1.append(special_point)
                flag = 1
            # print('sb', new_points[i])
            # print('ss',sub_point1)
        else:
            sub_point2.append(new_points[i])
            if new_points[i] == flag_point and i != len(new_points) - 1:
                sub_point2.append(special_point)
                flag = 2
    #
    # print('sub1', sub_point1)
    # print('sub2', sub_point2)
    if not sub_point1 or not sub_point2:
        return None, None
    sub_point1 = sort_points(sub_point1)
    sub_point2 = sort_points(sub_point2)
    if (flag == 1):
        sub_point1.remove(center)
    elif (flag == 2):
        sub_point2.remove(center)

    return sub_point1, sub_point2

def plan (points):
    points.append(points[0])
    index = checkangle(points)
    if not index:
        print('function checkangle return None')
        return None, None
    if index[0] == len(points) - 1   :
        index = [0]
    del points[-1]
    center = points[index[0]]
    inter_P = find_intersect_point(points, index[0])
    # print(inter_P)
    if not inter_P:
        print('function find_intersect_point return None')
        return None, None


    sub_point1, sub_point2 = take_sub_Points(points, center, inter_P[0])
    sub_point3, sub_point4 = take_sub_Points(points, center, inter_P[1])
    if not sub_point1 or not sub_point3:
        print('function take_sub_Points return None')
        return None, None

    s1 = calculate_polygon_area(sub_point1)
    s2 = calculate_polygon_area(sub_point2)
    s3 = calculate_polygon_area(sub_point4)
    s4 = calculate_polygon_area(sub_point3)


    if(abs(s1-s2) > abs(s3-s4)):
        return sub_point1, sub_point2
    elif(abs(s1-s2) < abs(s3-s4)):
        return sub_point3, sub_point4
    else:
        print('Can not compare S')
        return None

#non convex has a angle > 180
K = [[59, -40],[-10,-40],[-20, -85], [-63, 24], [-31, 56], [52, 26]]
#K2 = [ [59, -40],[-10,-40],  [-63, 24],[-31, 56] , [52, 26],[-20,0]]
#K2 = [ [59, -40],[-10,-40],  [-63, 24],[-31, 56] ,[-20,0], [52, 26]]
#K2 = [ [59, -40],[-10,-40],  [-63, 24],[-20,0],[-31, 56] , [52, 26]]
#K2 = [ [59, -40],[-10,-40],[-20,0],  [-63, 24],[-31, 56] , [52, 26]]
K2 = [ [59, -40],[40, -20], [-10,-40],  [-63, 24],[-31, 56] , [52, 26]]
K2 = [ [0, 4],[3,-1],  [1, -1],[-2, -2] ]
K2 = [[0, 0], [3, 2], [6, -3], [4,-3], [1, -4]]
K2 = [[0, -1], [-2, 1], [-1, 2], [-1, 3], [3, 1] ]
K2 =[[-4, -4], [-6, 0], [-2, 4], [-8, 6], [5, 10], [10, 8], [8,2], [3, -4]]

# print('K2', K2)
points = K2
ox,oy= zip(*K2)
center = [-10, -40]
intersect_point = [-38,-40]
a, b = plan(points)
plt.figure()
plt.plot(ox, oy,'-xk',label = 'range')
print(a)
print(b)
# a.append(a[0])
# b.append(b[0])
if a:
    oxa,oya= zip(*a)
    oxb,oyb= zip(*b)
    plt.plot(oxa, oya )
    plt.plot(oxb, oyb )
plt.show()