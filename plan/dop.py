import math
import numpy as np

class cost_function :

    def next_position (self, a, b) : # a = azimuth b = estimated position
        next_position = []
        theta = [90,180,270,360] #original[10,30,60,90]
        dd = [10, 20, 30, 40, 50]

        #a_i = np.tan(a)
        a_i = a*180/np.pi
        #print('azimuth (deg)= ')
        #print(a_i)
        #b[2]
        
        #print(b)
        
        for j in range(5):
            for i in range(4):

                ############ Method 1 #############
                if a_i > 0 :
                    beta = theta[i] + (90 - a_i)
                    #beta = theta[i] + 90 
                    beta_rad = beta * np.pi/180
                    tanb = np.tan(beta_rad)
                    # first esimation value
                    c = b[0] - tanb * b[1]
                    k = np.sqrt(np.square(dd[j]) / (np.square(tanb) + 1))

                    if tanb < 0 :
                        N = b[0] + tanb*k
                        #E = tanb*b[1] + 1*k
                        E = b[1] + 1*k
                        next_position.append([N, E, b[2]])
                    else :
                        N = b[0] - tanb*k
                        E = b[1] - 1*k
                        next_position.append([N, E, b[2]])
                else :
                    # beta = a_i + 90
                    beta = 90 - theta[i]
                    beta_rad = beta*np.pi/180
                    tanb = np.tan(beta_rad)
                    c = b[0] - tanb*b[1]
                    k = np.sqrt(np.square(dd[j]) / (np.square(tanb) + 1))

                    N = b[0] - tanb*k
                    E = b[1] - 1*k
                    next_position.append([N, E, b[2]])

                ############ Method 2 #############
                # b_norm = np.sqrt(np.square(b[0])+np.square(b[1]))
                # ref = [[-b[0]/b_norm], [-b[1]/b_norm]]

                # ro = [[-np.sin(a_i+theta[i]), np.cos(a_i+theta[i])],[np.cos(a_i+theta[i]), np.sin(a_i+theta[i])]]
                # v0 = np.matmul(ro, ref)
                # # print('v0 = ')
                # # print(v0)

                # N = b[0]+dd[j]*v0[0][0]
                # E = b[1]+dd[j]*v0[1][0]

                # next_position.append([N, E, b[2]])

        return next_position


    def calculate_dop(self, a, b, c):  # a=current position b=next position c=estimated position

        #print('--------------------')
        #print(a, b)
        r_1 = np.square(c[0]-a[0])+np.square(c[1]-a[1])
        br_1 = np.square(c[0]-a[0])+np.square(c[1]-a[1])+np.square(c[2]-a[2])

        r_2 = np.square(c[0]-b[0])+np.square(c[1]-b[1])
        br_2 = np.square(c[0]-b[0])+np.square(c[1]-b[1])+np.square(c[2]-b[2])

        H2 = [[-(c[1]-a[1])/r_1, (c[0]-a[0])/r_1, 0],[-(c[1]-b[1])/r_2, (c[0]-b[0])/r_2, 0]]
        H3 = [[((c[0]-a[0])*(c[2]-a[2]))/(br_1*np.sqrt(r_1)),((c[1]-a[1])*(c[2]-a[2]))/(br_1*np.sqrt(r_1)), -(np.sqrt(r_1)/np.sqrt(br_1))],[((c[0]-b[0])*(c[2]-b[2]))/(br_2*np.sqrt(r_2)),((c[1]-b[1])*(c[2]-b[2]))/(br_2*np.sqrt(r_2)), -(np.sqrt(r_2)/np.sqrt(br_2))]]
        H2.append(H3[0])
        H2.append(H3[1])
        H = H2
        #print("H = ")
        #print(H)

        Q = np.matmul(np.transpose(H), H)
        #print("Q = ")
        #print(Q)
        #print("trace Q = ")
        #print(np.trace(Q))
        GDOP = np.sqrt(np.trace(Q))
        D = np.linalg.det(Q)

        #print("GDOP = ")
        #print(GDOP)
        #print('--------------------')
        return GDOP, D

    


