import math
import numpy as np

#target position
#Target_n = 50
#Target_e = 20
#Target_d = 0
class least_square(object):
    def LeastQ(self,ob_point,a): # observation point ,a = azimuth

        H = [[np.tan(a[0]),-1],[np.tan(a[1]),-1]] #2*2
        b = [[ob_point[0][0]*np.tan(a[0])-ob_point[0][1]],[ob_point[1][0]*np.tan(a[1])-ob_point[1][1]]] #2*1

        # H = []
        # b = []

        # Node_number = len(ob_point)
        # for i in range(Node_number):
        #     H_i = [np.tan(a[i]),-1] #6*2
        #     b_i =[ob_point[i][0]*np.tan(a[i])-ob_point[i][1]] #6*1
        #     H.append(H_i)
        #     b.append(b_i)
        # print(H)
        # print(b)

        #H_H = [[np.square(np.tan(a[0]))+np.square(np.tan(a[1])), (np.tan(a[0])+np.tan(a[1]))], [(np.tan(a[0])+np.tan(a[1])), -2]]
        #H_b = [[np.tan(a[0])*(ob_point[0][0]*np.tan(a[0])-ob_point[0][1])+np.tan(a[1])*(ob_point[1][0]*np.tan(a[1])-ob_point[1][1])],[(ob_point[0][0]*np.tan(a[0])-ob_point[0][1])+(ob_point[1][0]*np.tan(a[1])-ob_point[1][1])]]
        
        H_H = np.matmul(np.transpose(H), H) #2*2
        H_b = np.matmul(np.transpose(H), b) #2*1
        
        Estimate= np.matmul(np.linalg.inv(H_H), H_b) #2*1
        Est_Target_n = Estimate[0][0]
        Est_Target_e = Estimate[1][0]
        Est_Target_d = 0
        # print('Estimate = ')
        # print(Estimate)
        # print('Est_Target_n')
        # print(Est_Target_n)

        return Est_Target_n,Est_Target_e,Est_Target_d
    def LeastQ_m(self,ob_point,a): # observation point ,a = azimuth

        #print(ob_point)
        #H = [[np.tan(a[0]),-1],[np.tan(a[1]),-1],[np.tan(a[2]),-1], [np.tan(a[3]),-1], [np.tan(a[4]),-1]]
        #b = [[ob_point[0][0]*np.tan(a[0])-ob_point[0][1]],[ob_point[1][0]*np.tan(a[1])-ob_point[1][1]], [ob_point[2][0]*np.tan(a[2])-ob_point[2][1]], [ob_point[3][0]*np.tan(a[3])-ob_point[3][1]], [ob_point[4][0]*np.tan(a[4])-ob_point[4][1]]]

        H = []
        b = []

        Node_number = len(ob_point)
        for i in range(Node_number):
            # 1
            # H_i = [np.tan(a[i]),-1] #6*2
            # b_i =[ob_point[i][0]*np.tan(a[i])-ob_point[i][1]] #6*1
            # 2
            H_i = [np.sin(a[i]),-np.cos(a[i])] #6*2
            b_i =[ob_point[i][0]*np.sin(a[i])-ob_point[i][1]*np.cos(a[i])] #6*1

            H.append(H_i)
            b.append(b_i)
            
        # print(H)
        # print(b)

        H_H = np.matmul(np.transpose(H), H) #2*2
        H_b = np.matmul(np.transpose(H), b) #2*1
        Estimate= np.matmul(np.linalg.inv(H_H), H_b) #2*1
        Est_Target_n = Estimate[0][0]
        Est_Target_e = Estimate[1][0]
        Est_Target_d = 0
        # print('Estimate = ')
        # print(Estimate)
        # print('Est_Target_n')
        # print(Est_Target_n)

        return Est_Target_n,Est_Target_e,Est_Target_d

    def LeastQ_3(self,ob_point,a): # observation point ,a = azimuth

        #print(ob_point)
        H = [[np.tan(a[0]),-1],[np.tan(a[1]),-1],[np.tan(a[2]),-1]] #3*2
        b = [[ob_point[0][0]*np.tan(a[0])-ob_point[0][1]],[ob_point[1][0]*np.tan(a[1])-ob_point[1][1]], [ob_point[2][0]*np.tan(a[2])-ob_point[2][1]]] #3*1

        # target position
        #Target_n = 50
        #Target_e = 20
        #Target_d = 0

        # H = []
        # b = []

        # Node_number = len(ob_point)
        # for i in range(Node_number):
        #     H_i = [np.tan(a[i]),-1] #6*2
        #     b_i =[ob_point[i][0]*np.tan(a[i])-ob_point[i][1]] #6*1
        #     H.append(H_i)
        #     b.append(b_i)
        # print(H)
        # print(b)

        H_H = [[np.square(np.tan(a[0]))+np.square(np.tan(a[1]))+np.square(np.tan(a[2])), (np.tan(a[0])+np.tan(a[1])+np.tan(a[2]))], [(np.tan(a[0])+np.tan(a[1])), -3]]
        H_b = [[np.tan(a[0])*(ob_point[0][0]*np.tan(a[0])-ob_point[0][1])+
        np.tan(a[1])*(ob_point[1][0]*np.tan(a[1])-ob_point[1][1])+
        np.tan(a[2])*(ob_point[2][0]*np.tan(a[2])-ob_point[2][1])],
        [(ob_point[0][0]*np.tan(a[0])-ob_point[0][1])+
        (ob_point[1][0]*np.tan(a[1])-ob_point[1][1])+
        (ob_point[2][0]*np.tan(a[2])-ob_point[2][1])]]
        
        #H_H = np.matmul(np.transpose(H), H) #2*2
        #H_b = np.matmul(np.transpose(H), b) #2*1
        Estimate= np.matmul(np.linalg.inv(H_H), H_b) #2*1
        Est_Target_n = Estimate[0][0]
        Est_Target_e = Estimate[1][0]
        Est_Target_d = 0
        # print('Estimate = ')
        # print(Estimate)
        # print('Est_Target_n')
        # print(Est_Target_n)

        return Est_Target_n,Est_Target_e,Est_Target_d

