import pygame
import math
import sys


def distance(A,B):
    return (((B[0]-A[0])**(2))+((B[1]-A[1])**(2)))**(1/2)


class Road: 
    def __init__(self, start, end, vehicles=[]):
        self.start=start
        self.end=end
        self.vehicles=vehicles
        self.length=distance(self.end,self.start)
        self.sin=(self.end[1]-self.start[1])/self.length
        self.cos=(self.end[0]-self.start[0])/self.length

    def host_vehicle(self, listing):
        if type(listing)==list:
            self.vehicles=self.vehicles + listing
        else:
            self.vehicles=self.vehicles + [listing]

    def kick_vehicle(self,vh):
        self.vehicles.remove(vh)


def beziercurve(start, end, control, res=15):
    L=[]
    for i in range(res):
        t=(i)/res
        k=(i+1)/res
        #p1=(((1-t)**2)*start[0]+(2*(1-t)*t)*control[0]+((t)**2)*end[0],((1-t)**2)*start[1]+(2*(1-t)*t)*control[1]+((t)**2)*end[1])
        #p2=(((1-k)**2)*start[0]+(2*(1-k)*k)*control[0]+((k)**2)*end[0],((1-k)**2)*start[1]+(2*(1-k)*k)*control[1]+((k)**2)*end[1])
        p1=(int(((1-t)**2)*start[0]+(2*(1-t)*t)*control[0]+((t)**2)*end[0]),int(((1-t)**2)*start[1]+(2*(1-t)*t)*control[1]+((t)**2)*end[1]))
        p2=(int(((1-k)**2)*start[0]+(2*(1-k)*k)*control[0]+((k)**2)*end[0]),int(((1-k)**2)*start[1]+(2*(1-k)*k)*control[1]+((k)**2)*end[1]))
        L.append(Road(p1,p2))
    return L


def contactpoint(r1, r2):
    if r1.start==r2.start or r1.start==r2.end:
        return r1.start
    else:
        return r1.end


class Vehicle:
    def __init__(self, path, timing, color=[255,0,255], configuration={"a0i":1.44, "bi":4.61, "v0i":16.6, "s0i":40,"Ti":1,"delta":4,"l":40}):
        self.path=path
        self.timing=timing
        self.color=color
        self.configuration=configuration
        self.current_road=path[0]
        if path[0].end==contactpoint(path[0],path[1]):
            self.position=self.current_road.start
        else:
            self.position=self.current_road.end
        self.v=0
        self.a=configuration["a0i"]

    def updatevp(self, dt=0, lead=None):
        if lead==None:
            self.a=self.configuration["a0i"]*(1-(self.v/self.configuration["v0i"])**(self.configuration["delta"]))
            if self.v + self.a*dt < 0:
                dxpn1=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dypn1=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] + dxpn1, self.position[1] + dypn1]
                self.v=0

            else:
                self.v=self.v+dt*self.a
                dxpn2=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dypn2=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] + dxpn2, self.position[1] + dypn2]
            
        else:
            s=distance(self.position, lead.position)-lead.configuration["l"]
            deltav=abs(self.v-lead.v)
            distr=(self.configuration["Ti"])*(self.v)
            distj=((self.v)*(deltav))/(2*((self.configuration["a0i"]*self.configuration["bi"])**(1/2)))
            sd=self.configuration["s0i"] + distr + distj
            self.a=self.configuration["a0i"]*(1-(self.v/self.configuration["v0i"])**(self.configuration["delta"])-(sd/s)**(2))
            if self.v + self.a*dt < 0:
                dxpl1=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dypl1=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] + dxpl1, self.position[1] + dypl1]
                self.v=0

            else:
                self.v=self.v+dt*self.a
                dxpl2=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dypl2=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] + dxpl2, self.position[1] + dypl2]
    
    def updatevm(self, dt, lead=None):
        if lead==None:
            self.a=self.configuration["a0i"]*(1-(self.v/self.configuration["v0i"])**(self.configuration["delta"]))
            if self.v + self.a*dt < 0:
                dxmn1=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dymn1=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] - dxmn1, self.position[1] - dymn1]
                self.v=0

            else:
                self.v=self.v+dt*self.a
                dxmn2=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dymn2=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] - dxmn2, self.position[1] - dymn2]
            
        else:
            s=distance(self.position, lead.position)-lead.configuration["l"]
            deltav=abs(self.v-lead.v)
            distr=(self.configuration["Ti"])*(self.v)
            distj=((self.v)*(deltav))/(2*((self.configuration["a0i"]*self.configuration["bi"])**(1/2)))
            sd=self.configuration["s0i"] + distr + distj
            self.a=self.configuration["a0i"]*(1-(self.v/self.configuration["v0i"])**(self.configuration["delta"])-(sd/s)**(2))

            if self.v + self.a*dt < 0:
                dxml1=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dyml1=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] - dxml1, self.position[1] - dyml1]
                self.v=0

            else:
                self.v=self.v+dt*self.a
                dxml2=dt*self.v*self.current_road.cos + ((dt**2)/2)*self.a*self.current_road.cos
                dyml2=dt*self.v*self.current_road.sin + ((dt**2)/2)*self.a*self.current_road.sin
                self.position=[self.position[0] - dxml2, self.position[1] - dyml2]


def outofbound(vh, road):
    cd1=vh.position[0]>=road.start[0] and vh.position[0]<=road.end[0]
    cd2=(vh.position[1]>=road.start[1] and vh.position[1]<=road.end[1])
    cd3=(vh.position[1]<=road.start[1] and vh.position[1]>=road.end[1])
    if (cd1) and (cd2 or cd3):
        return False
    else:
        return True


class Simulation:
    def __init__(self, dt):
        self.list_of_roads=[]
        self.list_of_vehicles=[]
        self.sprited=[]
        self.dt=dt
        self.time=0

    def create_road(self, begining, ending):
        self.list_of_roads=self.list_of_roads+[Road(begining, ending)]

    def create_roads(self, list_road):
        for rd in list_road:
            self.create_road(rd.start,rd.end)

    def input_vehicle(self, vehicule):
        self.list_of_vehicles=self.list_of_vehicles+[vehicule]
        self.sprited=self.sprited+[vehicule]
        r=vehicule.current_road
        r.host_vehicle(vehicule)

    def input_vehicles(self, list_vehicles):
        for vh in list_vehicles:
            self.input_vehicle(vh)

    def updatesim(self):
        LV=self.list_of_vehicles
        for vh in LV:
            if self.time>vh.timing:
                L=vh.path
                indexr=L.index(vh.current_road)
                sib=vh.current_road.vehicles
                if len(sib[:sib.index(vh)])>0:
                    if indexr<len(L)-1:
                        if L[indexr].end==contactpoint(L[indexr],L[indexr+1]):
                            vh.updatevp(self.dt,sib[sib.index(vh)-1])
                        elif L[indexr].start==contactpoint(L[indexr],L[indexr+1]):
                            vh.updatevm(self.dt,sib[sib.index(vh)-1])
                    else:
                        if L[indexr].start==contactpoint(L[indexr],L[indexr-1]):
                            vh.updatevp(self.dt,sib[sib.index(vh)-1])
                        elif L[indexr].end==contactpoint(L[indexr],L[indexr-1]):
                            vh.updatevm(self.dt,sib[sib.index(vh)-1])
                else:
                    if indexr<len(L)-1:
                        if L[indexr].end==contactpoint(L[indexr],L[indexr+1]):
                            vh.updatevp(self.dt)
                        elif L[indexr].start==contactpoint(L[indexr],L[indexr+1]):
                            vh.updatevm(self.dt)
                    else:
                        if L[indexr].start==contactpoint(L[indexr],L[indexr-1]):
                            vh.updatevp(self.dt)
                        elif L[indexr].end==contactpoint(L[indexr],L[indexr-1]):
                            vh.updatevm(self.dt)

                if outofbound(vh, vh.current_road):
                    indexr=indexr+1
                    if indexr<len(L):
                        vh.current_road=L[indexr]
                        r=L[indexr]
                        r.host_vehicle(vh)
                        s=L[indexr-1]
                        s.kick_vehicle(vh)
                        vh.position=contactpoint(r,s)
                    else:
                        vh.position[0]=1920
                        vh.position[1]=1080
                        self.list_of_vehicles.remove(vh)
                        p=L[indexr-1]
                        p.kick_vehicle(vh)
        self.time=self.time+self.dt


class Window:
    def __init__(self, sim, config={"size":[1280, 720], "fps":60}):
        self.sim=sim
        self.size=config["size"]
        self.fps=config["fps"]

    def loop(self):
        pygame.init()
        screen = pygame.display.set_mode(self.size)
        color1= 255,255,255
        color2= 120,0,120
        black=0,0,0
        clock = pygame.time.Clock()
        R=[]
        L=[]
        S=self.sim.sprited
        for vh in S:
            vh_surface=pygame.Surface((40,8),pygame.SRCALPHA)
            vh_surface.fill(vh.color)
            vh_rect=vh_surface.get_rect(center = (vh.position[0]-40,vh.position[1]))
            L=L+[[vh_surface,vh_rect]]
            R=R+[[vh_surface,vh]]
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: sys.exit()
            screen.fill(black)
            for r in self.sim.list_of_roads:
                pygame.draw.line(screen,color1,r.start,r.end,11)
            
            for i in range(len(L)):
                vehi=S[i]
                sinu=vehi.current_road.sin
                L[i][1].x=vehi.position[0]
                L[i][1].y=vehi.position[1]
                R[i][0]=pygame.transform.rotate(L[i][0],-math.degrees(math.asin(sinu)))
                if self.sim.time>vehi.timing:
                    screen.blit(R[i][0],L[i][1])

            self.sim.updatesim()
            pygame.display.update()
            clock.tick(self.fps)