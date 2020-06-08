#if !defined(_GROUPMODE_H_INCLUDED_)
#define _GROUPMODE_H_INCLUDED_

#include <iostream>
#include<algorithm>   //����sort
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <map>
#include <iterator>   //
#include <set>        //�����������������󲢼�

using namespace std;

enum
{
	pointType_UNDO,
	pointType_NOISE,
	pointType_BORDER,
	pointType_CORE

};

class point {
public:
	double x;
	double y;
	int cluster;
	double carnumber;


	int pointType;  //1 noise 2 border 3 core
	int pts;        //points in MinPts 
	int corePointID;
	vector<int> corepts;
	int  visited;

	void init();
	point();
	point(double a, double b, int c, double d)
	{
		x = a;
		y = b;
		cluster = c;
		carnumber = d;
	};
};

//����ĺ��������������DBSCAN��point�ࡣ
double stringToFloat(string i);
vector<point> openFile(const char* dataset);
double Radians(double x);
double squareDistance(point a, point b);
vector<point> DBSCAN(vector<point> dataset, double Eps, int MinPts);

//����������Ĺ���
int findMaxCluster(vector<point> handledata);
struct ListNode
{
	vector<point> elements;
	vector<int> time;
	ListNode* next;
};
ListNode* CreatFirstNode(ListNode* head, vector<point> handledata, int times);  //������һ��Node
ListNode* CreatList(ListNode* head, int MaxCluster, vector<point> handledata, int times);//�������������
void ShowList(ListNode* head);  //��ʾ����

//��������ģʽ��������
int FindListLength(ListNode* head);   //�ҳ�������
int FindPointInList_Length(ListNode* list);    //�ҳ�������ÿ����������point�ĸ���
int FindCommonNumsInTwoListElements(ListNode* list1, ListNode* list2);   //�ҳ�������������������������ͬӵ�еĳ��ŵ�����
bool JusticeIfElementsInFirstListBelongToSecondList(ListNode* list1, ListNode* list2);   //��֤����1��point����������2�е�point
ListNode* FindCommonPointsAndBothTimesInTwoListElements(ListNode* head, ListNode* list1, ListNode* list2, int times);//����һ��
														//ȡ����list��elements������time�Ĳ������������ֵ��һ���µ�list��Ԫ
void InsertCanList(ListNode* newnode, ListNode* nowlist, int length);   //�ж��Ƿ���Լ����µ����ӣ���newnode��
bool JusticeIfELementsInTwoListAreSame(ListNode* list1, ListNode* list2);   //�ж��������ӵ�elements�Ƿ�����ȫ��ͬ��
vector<int> GetAllTimeInTwoList(ListNode* newnode, ListNode* diferencelist);  //��ȡ��������ʱ�䲢��
ListNode* GroupMode(ListNode* originalhead, ListNode* freshhead, int times);    //����Ҫ�ĺ���
void ShowAndWriteResult(ListNode* result);     //�����д���ļ���
void ShowNumsOfList(ListNode* result);        //���������з��˶��ٸ���







#endif
