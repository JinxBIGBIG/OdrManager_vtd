#define WIN32  
#include "posInfo.hpp"
#include <iostream>
#include <time.h> //timeval
#include <windows.h>
#include<string.h>
#include<vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include<string>
#include<iostream>
#include<map>
using namespace std;
//

int main(int argc, char** argv) {

    std::string odrFile = "F:\\111\\evaluate\\roadInfos\\data\\Germany_2018.xodr";
    //std::string contact_path = "F:\\111\\evaluate\\roadInfos\\data\\contact_point.csv";

    getPosInfo posManager;
    bool ret = posManager.init(odrFile);
    if (!ret) {
        std::cout << "load xodr file error!" << std::endl;
        return -1;
    }



    string file_name = "F:\\111\\test_lane_4.csv";
    /*  string file_path = "F:\\i - Evaluator\\data";*/
    /*  file_name = file_path + file_name;*/

    std::cout << file_name << std::endl;

    std::vector<std::vector<double>> data_column;

    vector<double> simTime;
    vector<double> simFrame;
    vector<double> RoadData;
    vector<double> PlayerID;
    vector<double> posX;
    vector<double> posY;
    vector<double> pointID;

    double m = 0;   //���빫������ʱ��
    double n = 0;   //�뿪��������ʱ��
    double p = 0;   //ѹ��time
    double q = 0;   //����ѹ��time
    vector<double> m_g;
    vector<double> n_g;
    vector<double> bg_t;
    vector<double> ed_t;
    vector<double> p_g;
    //vector<double> q_g;

    vector<double>  O_soline;   //����ѹ��ʱ��
    vector<double>  I_soline;   //��ʼѹ��ʱ��
    vector<double>  O_busline;   //����ѹ��ʱ��
    vector<double>  I_busline;   //��ʼѹ��ʱ��

    ifstream fp(file_name);
    string line;
    getline(fp, line);
    while (getline(fp, line))
    {
        vector<double> data_line;
        string number;
        istringstream readstr(line);
        for (int j = 0; j < 7; j++)
        {
            getline(readstr, number, ',');
            data_line.push_back(stof(number.c_str()));
        }
        if (!data_line.empty())
        {
            simTime.push_back(data_line[0]);
            simFrame.push_back(data_line[1]);
            RoadData.push_back(data_line[2]);
            PlayerID.push_back(data_line[3]);
            posX.push_back(data_line[4]);
            posY.push_back(data_line[5]);
            pointID.push_back(data_line[6]);
        }

    }

    data_column.push_back(simTime);
    data_column.push_back(simFrame);
    data_column.push_back(RoadData);
    data_column.push_back(PlayerID);
    data_column.push_back(posX);
    data_column.push_back(posY);
    data_column.push_back(pointID);
    //int f = simTime.size();
    //cout << "������" << f << endl;
    //map<double, int>timeSt;
   // map<double, int>::iterator iter;

    for (int i = 0; i < simTime.size(); i++)
    {
        Point p1(data_column[4][i], data_column[5][i], 0);
        //std::cout << "x�����꣺" << data_column[4][i] << endl;
        //std::cout << "y�����꣺" << data_column[5][i] << endl;
        /*Point data_column[4][i];*/
        //bool isInBusLane = posManager.isInSpecialLaneType(p1, OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS);
        bool isOnRoadmark = posManager.isOnRoadMark(p1);
        cout << "ison:" << isOnRoadmark << endl;
        m_g.push_back(isOnRoadmark);

        //posManager.getInertialPosInfo(p1);
        //double speed = posManager.findLimitedSpeed();
        //cout << "���٣�" << speed << endl;
        //std::cout << "time:" << simTime[i] << endl;
       // timeSt[simTime[i]] = m_g[i];
    }
    double t;
    for (int m = 0; m < m_g.size() - 1; m++)
    {
        //cout << "m:" << m << endl;
        t = (data_column[0][m + 1] - data_column[0][m]);
        if (t >= 0.009 && t <= 0.011)
        {
            //double t = data_column[0][m];
            //cout << "sit:" << t << endl;
            cout << "��ײ��" << m_g[m] << endl;
            if ((m_g[m] == 1) || (m_g[m - 1] == 1) || (m_g[m - 2]) == 1 || (m_g[m - 3]) == 1)
            {
                n_g.push_back(simTime[m]);   //n_gΪѹ�߼���
            }
        }

    }
    if (n_g.size())
    {
        cout << "n_g:" << n_g.size() << endl;
        bg_t.push_back(n_g[0]);
        for (int c = 0; c < n_g.size() - 1; c++)
        {
            if ((n_g[c + 1] - n_g[c] > 0.011) || (n_g[c + 1] - n_g[c]) < 0.009)
            {

            }
            cout << "��ײtime��" << n_g[c] << endl;
        }
    }
    if (bg_t.size() && ed_t.size())
    {
        for (size_t i = 0; i < bg_t.size()-1; i++)
        {
            cout << "begin_t:" << bg_t[i] << endl;
            cout << "end_t:" << ed_t[i] << endl;
        }
    }

 /*   for (iter = timeSt.begin(); iter != timeSt.end(); iter++)
    {
        std::cout << iter->first << ' ' << iter->second << endl;
    }*/
    //    if (isOnRoadmark == 1)
    //    {
    //        //std::cout << "this point is in bus lane." << std::endl;
    //        std::cout << "ѹ��time��" << data_column[0][i] << endl;

    //        m = data_column[0][i];
    //        m_g.push_back(m);
    //    }
    //    else
    //    {
    //        n = data_column[0][i];
    //        n_g.push_back(n);
    //        //std::cout << "this point is not in bus lane." << std::endl;
    //        std::cout << "δѹ��time��" << data_column[0][i] << endl;
    //    }
    

//    if (m_g.size())
//    {
//        for (int j = 0; j < m_g.size()-1; j++)
//        {
//            if ((m_g[j + 1] - m_g[j]) != data_column[0][j + 3] - data_column[0][j])
//            {
//                double O_time = data_column[0][j] + (data_column[0][j + 3] - data_column[0][j]);  //ÿ���뿪����������time
//                O_busline.push_back(O_time);   //ʻ����������time-array
//                I_busline.push_back(data_column[0][j + 1]);  //���빫������time-array  
//            }
//        }
//    }
//    int h = I_busline.size();
//    vector<double>num(h, 1);  //���Ϲ����`
//    double* a1 = new double[I_busline.size()];
//    double* b1 = new double[O_busline.size()];
//    double* e1 = new double[I_busline.size()];
//
//    if (!I_busline.empty())									//��vector������ת������ͨ����
//    {
//        memcpy(a1, &I_busline[0], I_busline.size() * sizeof(double));
//    }
//    /*cout << a1[1] << endl;*/
//
//    if (!O_busline.empty())									//��vector������ת������ͨ����
//    {
//        memcpy(b1, &O_busline[0], O_busline.size() * sizeof(double));
//    }
//    /*cout << b1[1] << endl;*/
//    if (!num.empty())									//��vector������ת������ͨ����
//    {
//        memcpy(e1, &num[0], num.size() * sizeof(double));
//    }
//    for (int m = 0; m < h; m++)
//    {
//        cout << "a1:" << a1[m] << endl;
// }
//    cout << "���Ϲ������" << h << endl;
//    int pp = sizeof(a1) / sizeof(double*);
//    cout << "������������ʱ�̵ĳ��ȣ�" << sizeof(a1)/sizeof(double*) << endl;
//    cout << "����������ʼ����ʱ�̵ĳ��ȣ�" <<sizeof(a1)/sizeof(double) << endl;
//    cout << "δѹ�߼��ϳ��ȣ�" << n_g.size() << endl;
//    
    return 0;
}
//  
//
//int main()
//{
//        cout<<  string(" {\"state\":\"0\",\"valid_state\":\"0\",\"ego_speed_max\":\"86\",\"crash_state\":\"0\",\
//        \"crash_time\":\"10\",\"average_state\":\"0\",\"average_score\":\"80\",\"ttc_state\":\"1\",\"ttc_score\":\"90\",\
//        \"ttc_min\":\"4.45\",\"ttc_time\":\"4.245\",\"ttc_count\":\"10000\",\"thw_state\":\"0\",\"thw_score\":\"70\",\
//    \"thw_min\":\"4.45\",\"thw_time\":\"4.5\",\"thw_count\":\"10000\"\
//  }");
//        return 0;
//}
//
////
////int main()
////{
////	std::string odrFile = "F:\\111\\evaluate\\roadInfos\\data\\Germany_2018.xodr";
////    //std::string contact_path = "F:\\111\\evaluate\\roadInfos\\data\\contact_point.csv";
////
////    getPosInfo posManager;
////    bool ret = posManager.init(odrFile);
////    if (!ret) {
////        std::cout << "load xodr file error!" << std::endl;
////        return -1;
////    }
////        Point p1(7218.191,-2915.432, 0);
////        cout << "x�����꣺" << 7218.191 << endl;
////        cout << "y�����꣺" << 2915.423 << endl;
////        Point p2(7218.314443, - 2914.360224, 0);
////        /*Point data_column[4][i];*/
////        //bool isInBusLane = posManager.isInSpecialLaneType(p1, OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS);
////        bool isOnRoadmark = posManager.isOnRoadMark(p1);
////        bool test2= posManager.isOnRoadMark(p2);
////        cout << "�Ƿ�ѹ��p1��" << isOnRoadmark << endl;
////        cout << "�Ƿ�ѹ��p2��" << test2 << endl;
////
////}
