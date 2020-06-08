#if !defined(_GROUPMODE_H_INCLUDED_)
#define _GROUPMODE_H_INCLUDED_

#include <iostream>
#include<algorithm>   //排序sort
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
#include <set>        //这个和上面这个可以求并集

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

//下面的函数用于制造符合DBSCAN的point类。
double stringToFloat(string i);
vector<point> openFile(const char* dataset);
double Radians(double x);
double squareDistance(point a, point b);
vector<point> DBSCAN(vector<point> dataset, double Eps, int MinPts);

//下面是链表的构造
int findMaxCluster(vector<point> handledata);
struct ListNode
{
	vector<point> elements;
	vector<int> time;
	ListNode* next;
};
ListNode* CreatFirstNode(ListNode* head, vector<point> handledata, int times);  //创建第一个Node
ListNode* CreatList(ListNode* head, int MaxCluster, vector<point> handledata, int times);//创建后面的链表
void ShowList(ListNode* head);  //显示链表；

//下面是组模式分析函数
int FindListLength(ListNode* head);   //找出链表长度
int FindPointInList_Length(ListNode* list);    //找出链表中每个链子里面point的个数
int FindCommonNumsInTwoListElements(ListNode* list1, ListNode* list2);   //找出两个链表中任意两个子链共同拥有的车号的数量
bool JusticeIfElementsInFirstListBelongToSecondList(ListNode* list1, ListNode* list2);   //验证链子1中point都属于链子2中的point
ListNode* FindCommonPointsAndBothTimesInTwoListElements(ListNode* head, ListNode* list1, ListNode* list2, int times);//见下一行
														//取两个list的elements交集和time的并集，把这个赋值给一个新的list单元
void InsertCanList(ListNode* newnode, ListNode* nowlist, int length);   //判断是否可以加入新的链子，即newnode。
bool JusticeIfELementsInTwoListAreSame(ListNode* list1, ListNode* list2);   //判断两个链子的elements是否是完全相同的
vector<int> GetAllTimeInTwoList(ListNode* newnode, ListNode* diferencelist);  //获取两个子链时间并集
ListNode* GroupMode(ListNode* originalhead, ListNode* freshhead, int times);    //最主要的函数
void ShowAndWriteResult(ListNode* result);     //将结果写入文件中
void ShowNumsOfList(ListNode* result);        //看看链表中分了多少个簇







#endif
