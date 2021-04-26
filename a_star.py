import math

class Target:
    """
        由决策节点传给planning节点的数据封装
        x、y为移动目标的横、纵坐标
    """
    def __init__(self,x=0,y=0):
        self.x=x
        self.y=y

class Velocity:
    """
        需要传输给base节点的数据封装
        车辆移动的X,Y方向速度及角速度a
    """
    def __init__(self,x=0,y=0,a=0):
        self.x=x
        self.y=y
        self.a=a

class CarPosition: #车辆尺寸是 600mm X 450mm; 初始值设为各车起始位置
    """
        需要从感知节点得到的数据封装
        传入数据的格式是（车左上角点X坐标，车左上角点Y坐标，车长度，车宽度）
        ## 需要改进，现在只能车头朝左/朝右放置
    """
    def __init__(self,own=(0, 348,60,45),ally=(0,0,60,45),enemy1=(708,0,60,45),enemy2=(708,358,60,45)):
        self.own = own 
        self.ally = ally
        self.enemy1 = enemy1
        self.enemy2 = enemy2

class Array2D:
    """
        说明：
            1.构造方法需要两个参数，即二维数组的宽和高
            2.成员变量w和h是二维数组的宽和高
            3.使用：‘对象[x][y]’可以直接取到相应的值
            4.数组的默认值都是0
    """
    def __init__(self,w,h):
        self.w=w
        self.h=h
        self.data=[]
        self.data=[[0 for y in range(h)] for x in range(w)]
        self.obs=[] # 障碍物的位置列表
 
 
    def showArray2D(self):
        for y in range(self.h):
            for x in range(self.w):
                if(self.data[x][y] == 0): # 无障碍空间
                    print(" ",end=' ')
                elif(self.data[x][y] == 1): # 固定障碍空间
                    print("X", end=' ')
                elif(self.data[x][y] == 8): # 规划行进路线
                    print("@", end=' ')
                else: # 车辆
                    print("&", end=' ') 
                # print(self.data[x][y],end=' ')
            print("")
 
    def __getitem__(self, item):
        return self.data[item]

    # 在地图上添加以（X,Y）作为左上角顶点的矩形障碍，长度为w，宽度为y，在map2d中用1表示
    def add_block(self,x,y,w,h):
        for i in range(x,x+w+1):
            for j in range(y,y+h+1):
                self.data[i][j] = 1
                self.obs.append((i,j))

    # 在地图上添加以（X,Y）作为左上角顶点的矩形车辆，长度为w，宽度为y，在map2d中用2表示
    def add_car(self,x,y,w,h):
        for i in range(x,x+w+1):
            for j in range(y,y+h+1):
                self.data[i][j] = 2
                self.obs.append((i,j))


class Point:
    """
    表示一个点
    """
    def __init__(self,x,y):
        self.x=x;self.y=y
 
    def __eq__(self, other):
        if self.x==other.x and self.y==other.y:
            return True
        return False
    def __str__(self):
        return "x:"+str(self.x)+",y:"+str(self.y)


class APF:
    """
    APF Artificial Potential Field 人工势场法（局部路径规划）
    """
    def __init__(self, start, end, map, obs):
        # start end : inserted list
        # obs : list of 2-element list
        self.field = map
        self.ori = start  # start point
        self.current = start  # current position
        self.tar = end  # target point
        self.obs = obs
        # for sub in obs:
        #     # mark all the obs
        #     # EDGE ONLY
        #     self.field[sub[0]][sub[1]] = 1
        self.G = 10

    def dist(self, pos, source):  # (current pos,gravity source)
        distance = ((pos[0]-source[0])**2+(pos[1]-source[1])**2)**0.5
        return distance

    def sinAngle(self, pos, source):
        return (pos[1]-source[1])/self.dist(pos, source)

    def cosAngle(self, pos, source):
        return (pos[0]-source[0])/self.dist(pos, source)

    def currentForce(self):
        # fource summation
        forceX = 0
        forceY = 0
        for sub in self.obs:
            #obsticles results in minus force
            forceX += -(self.G*1/(self.dist(self.current, sub)
                                  ** 2))*self.cosAngle(self.current, sub)
            forceY += -(self.G*1/(self.dist(self.current, sub)
                                  ** 2))*self.sinAngle(self.current, sub)
        #target results in positive force
        forceX += (self.G*1/(self.dist(self.current, self.tar)
                             ** 2))*self.cosAngle(self.current, self.tar)
        forceY += (self.G*1/(self.dist(self.current, self.tar)
                             ** 2))*self.sinAngle(self.current, self.tar)
        self.xf = forceX
        self.yf = forceY


class AStar:
    """
    AStar算法的Python3.x实现（全局路径规划）
    """
 
    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y)) * 10  # 计算h值
 
    def __init__(self, map2d, startPoint, endPoint, passTag=0):
        """
        构造AStar算法的启动条件
        :param map2d: Array2D类型的寻路数组
        :param startPoint: Point或二元组类型的寻路起点
        :param endPoint: Point或二元组类型的寻路终点
        :param passTag: int类型的可行走标记（若地图数据!=passTag即为障碍）
        """
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 寻路地图
        self.map2d = map2d
        # 起点终点
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)
 
        # 可行走标记
        self.passTag = passTag
 
    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        :return: Node
        """
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode
 
    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False
 
    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None
 
    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None
 
    def searchNear(self, minF, offsetX, offsetY):
        """
        搜索节点周围的点
        :param minF:F值最小的节点
        :param offsetX:坐标偏移量
        :param offsetY:
        :return:
        """
        # 越界检测
        if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.w - 1 or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.h - 1:
            return
        # 如果是障碍，就忽略
        if self.map2d[minF.point.x + offsetX][minF.point.y + offsetY] != self.passTag:
            return
        # 如果在关闭表中，就忽略
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
        if self.pointInCloseList(currentPoint):
            return
        # 设置单位花费
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14
        # 如果不再openList中，就把它加入openlist
        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:
            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + step
            currentNode.father = minF
 
    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 判断寻路终点是否是障碍
        if self.map2d[self.endPoint.x][self.endPoint.y] != self.passTag:
            return None
 
        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)
        # 2.主循环逻辑
        while True:
            # 找到F值最小的点
            minF = self.getMinNode()
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF)
            self.openList.remove(minF)
            # 判断这个节点的上下左右节点
            self.searchNear(minF, 0, -1)
            self.searchNear(minF, 0, 1)
            self.searchNear(minF, -1, 0)
            self.searchNear(minF, 1, 0)
            self.searchNear(minF, 1, -1)
            self.searchNear(minF, 1, 1)
            self.searchNear(minF, -1, 1)
            self.searchNear(minF, -1, 1)
            # 判断是否终止
            point = self.endPointInCloseList()
            if point:  # 如果终点在关闭表中，就返回结果
                # print("关闭表中")
                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point)
                        cPoint = cPoint.father
                    else:
                        # print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())
                        return list(reversed(pathList))
            if len(self.openList) == 0:
                return None
    
class Planner():

    """
        planner 节点的类
        包括各路径规划函数、地图初始化、地图更新函数的封装
    """

    def __init__(self,ratio=0.1):
        self.map2d = Array2D(math.ceil(809*ratio),math.ceil(449*ratio))
        self.ratio = ratio

    """
        初始化地图
        ratio为调整地图精确度的比例，地图初始比例是1像素（一个二维数组里的数字）=1cm * 1cm
        ratio默认值设为0.1是便于在终端现实整个地图
    """
    def map_initialization(self):
        # 创建一个808*ratio X 449*ratio的比例缩小/放大地图;（实际尺寸8080mm X 4480mm)
        self.map2d = Array2D(math.ceil(809*self.ratio),math.ceil(449*self.ratio))
        #设置固定障碍(依据比赛地图设置了除了B5之外的固定障碍物)
        blocks = [(0,100,100,20),(150,214,80,20),(150,348,20,100),(354,93,100,20), \
            (354,334,100,20),(638,0,20,100),(578,214,80,20),(708,328,100,20)]
        for i in blocks:
            self.map2d.add_block(math.floor(i[0]*self.ratio),math.floor(i[1]*self.ratio),math.floor(i[2]*self.ratio),\
                math.floor(i[3]*self.ratio))
        #显示地图当前样子


    """
        依据其他车辆位置实时更新地图
        ratio为调整地图精确度的比例，地图初始比例是1像素（一个二维数组里的数字）=1cm * 1cm
        ratio默认值设为0.1是便于在终端现实整个地图
    """
    def update_map(self,other_cars):
        #更新障碍
        blocks = [other_cars.ally, other_cars.enemy1, other_cars.enemy2]
        for i in blocks:
            self.map2d.add_car(math.floor(i[0]*self.ratio),math.floor(i[1]*self.ratio),math.floor(i[2]*self.ratio),\
                math.floor(i[3]*self.ratio))


    """
        依据当前时刻的地图，全局规划路径
        规划起点为(x1,y1)终点为(x2,y2)
        ratio为调整地图精确度的比例，地图初始比例是1像素（一个二维数组里的数字）=1cm * 1cm
        ratio默认值设为0.1是便于在终端现实整个地图
    """
    def astar_plan(self,x1,y1,x2,y2):
        #创建AStar对象,并设置起点为(x1,y1)终点为(x2,y2)
        aStar=AStar(self.map2d,Point(math.floor(x1*self.ratio),math.floor(y1*self.ratio)),Point(math.floor(x2*self.ratio),\
            math.floor(y2*self.ratio)))
        #开始寻路
        pathList=aStar.start()
        #遍历路径点,在map2d上以'8'显示
        if pathList==None:
            print("Start poin or end poin is blocked by obstacle")
        else:
            for point in pathList:
                self.map2d[point.x][point.y]=8
        return pathList

    """
        依据当前时刻的地图，局部规划路径
        将Astar规划的路线分为五等分的小段，每次APF给出完成该小段路径的速度指令
    """
    def APF_plan(self,pathList,j):
        MAX_FB_SPEED = 3 # 最大前进(forward/backward)速度 3m/s
        MAX_LR_SPPED = 2 # 最大平移(left/right)速度 2m/s

        current_x = pathList[math.floor(pathList.__len__()/5)*j].x # 及时从感知节点获取实时位置，模拟时使用上一次规划时车的位置
        current_y = pathList[math.floor(pathList.__len__()/5)*j].y # 及时从感知节点获取实时位置，模拟时使用上一次规划时车的位置
        target_x = pathList[math.floor((pathList.__len__()-1)/5)*(j+1)].x
        target_y = pathList[math.floor((pathList.__len__()-1)/5)*(j+1)].y

        apf = APF((current_x,current_y),(target_x,target_y),self.map2d,self.map2d.obs)
        apf.currentForce()
        
        # 参数使用上可能需要再做处理，单位上需要和base node统一
        v_x = apf.xf * MAX_FB_SPEED
        v_y = apf.yf * MAX_LR_SPPED
        while abs(v_x) >= 3 or abs(v_y) >= 2: # 保持前进方向不变的前提下，让速度在最大值范围内尽可能大
            v_x = v_x * 0.95
            v_y = v_y * 0.95
        velocity = Velocity(v_x,v_y)
        return velocity


################################
################################

"""
    关于地图的说明
    X 是静态障碍物的位置
    @ 是规划的路径
    & 是除自身之外的其他车辆
"""
"""
    ## APF的更新频率要高于Astar的更新频率
    # APF在每一次Astar给出路线后，APF更新5次（暂定）
    # APF更新频率与planning节点给base节点发送速度指令的频率
    # Astar更新频率与决策节点给planning节点发送目的地坐标的频率、感知节点给planning节点车辆位置信息的频率、地图更新频率相同相同
"""

if __name__ == '__main__':
    ## DEMO说明
    ###########################
    ## 决策节点第一次传输目标位置
    CarPosition = CarPosition() 
    planner = Planner()
    planner.map_initialization() # 初始化静态障碍物
    planner.update_map(CarPosition) # 依据四个车辆的位置，更新地图
    pathList = planner.astar_plan(0,200,300,100) # astar规划出全局路径
    print("--"*80)
    #显示地图当前样子
    planner.map2d.showArray2D()
    print("--"*80)
    for j in range (0,5):
        vel = Velocity()
        vel = planner.APF_plan(pathList,j) # APF给出局部移动的速度信息，将全局路径分为五段，依此次输出五个速度指令
        print("velocity-",j+1,":",vel.x,",",vel.y)

    ###########################
    ## 决策节点第二次传输目标位置
    # 更新其他三个车辆的位置信息
    CarPosition.ally = (200,300,60,45)
    CarPosition.enemy1 = (500,400,60,45)
    CarPosition.enemy2 = (700,150,60,45)
    planner.map_initialization() # 初始化静态障碍物
    planner.update_map(CarPosition) # 依据四个车辆的位置，更新地图
    pathList = planner.astar_plan(300,100,600,0) # astar规划出全局路径
    print("--"*80)
    #显示地图当前样子
    planner.map2d.showArray2D()
    print("--"*80)
    for j in range (0,5):
        vel = Velocity()
        vel = planner.APF_plan(pathList,j) # APF给出局部移动的速度信息，将全局路径分为五段，依此次输出五个速度指令
        print("velocity-",j+1,":",vel.x,",",vel.y)

"""
    需要改进的点：
    将自身体积融合到a*算法中
    将a*和APF更好的结合 & 调参数
    实现车辆自由位姿在地图上的呈现
    实现更加灵敏和精细的规划和移动
    需要处理角速度和角度偏转问题
"""
