#include "GROUPMODE.h"

const int minO = 5;
const double PIx = 3.141592653589793;
const double RADIUS = 6378.16;

void point::init()     //初始化point类型中的int参数
{
	cluster = 0;
	pointType = pointType_UNDO;//pointType_NOISE pointType_UNDO
	pts = 0;                   //points in MinPts 
	visited = 0;
	corePointID = -1;
}


double stringToDouble(string i)   //通过使用stringstream使得string类型的i可以转换为float类型
{
	stringstream sf;    //#include <sstream>包含的东西，
	double score = 0;
	sf << i;
	sf >> score;
	return score;
}


vector<point> openFile(const char* dataset)   //将txt数据以double的形式传入一个point类中，妙啊！
{
	//先打开文件
	fstream file;
	file.open(dataset, ios::in);
	if (!file)    //如果打开文件失败
	{
		cout << "Open File Failed!" << endl;
		vector<point> a;
		return a;
	}
	//传入文件
	vector<point> data;

	while (!file.eof()) {
		string temp;
		file >> temp;
		//int split = temp.find(',', 0);
		//point p(stringToFloat(temp.substr(0, split)), stringToFloat(temp.substr(split + 1, temp.length() - 1)), i++);
		//point p(stringToDouble(temp.substr(0, 13)), stringToDouble(temp.substr(15, 27)), 0, stringToDouble(temp.substr(29, temp.length() - 1)));

		point p(stringToDouble(temp.substr(0, 14)), stringToDouble(temp.substr(15, 13)), 0, stringToDouble(temp.substr(29, (temp.length() - 29))));
		data.push_back(p);
	}
	file.close();
	cout << "successful!" << endl;
	return data;
}
double Radians(double x)
{
	return x * PIx / 180;
}

double squareDistance(point a, point b)   //计算两个点之间的直线距离
{
	double dlon = Radians(b.x - a.x);
	double dlat = Radians(b.y - a.y);

	double middle = (sin(dlat / 2) * sin(dlat / 2)) + cos(Radians(a.y)) * cos(Radians(b.y)) * (sin(dlon / 2) * sin(dlon / 2));
	double angle = 2 * atan2(sqrt(middle), sqrt(1 - middle));
	return angle * RADIUS * 1000;
}

vector<point> DBSCAN(vector<point> dataset, double Eps, int MinPts)
{
	int clusterID = 0;

	int len = dataset.size();//数据长度
	for (int i = 0; i < len; i++)//参数初始化
	{
		dataset[i].init();
	}


	vector<vector <double>> distP2P(len);   //定义一个二维数组。用于放置每个点到其他点的距离。
	//计算每个点到其他点的距离小于规定距离Eps的个数；
	cout << "calculate pts" << endl;
	for (int i = 0; i < len; i++) {
		for (int j = 0; j < len; j++)
		{//i+1
			double distance = squareDistance(dataset[i], dataset[j]); //计算dataset里的每个point类到其他类的距离；
			distP2P[i].push_back(distance); //把每个距离放进一个新的二维数组distP2P中。第一维是类数，第二维是这个point类到其他类的距离集合。
			if (distance <= Eps)
			{
				dataset[i].pts++;
			}
		}
	}

	//找出corepoint，把这些点类放进一个一维数组之中。
	cout << "core point " << endl;
	vector<point> corePoint;
	for (int i = 0; i < len; i++) {
		int tempPts = dataset[i].pts;  //将每个点周围符合条件的点数放进tempPts之中
		if (tempPts >= MinPts) {       //符合条件的话
			dataset[i].pointType = pointType_CORE;   //设置这个点类的pointType和corePointID
			dataset[i].corePointID = i;
			corePoint.push_back(dataset[i]);//并且将这些点类返回一个corePoint数组。
		}
	}
	cout << "joint core point" << endl;

	//joint core point
	int numCorePoint = corePoint.size(); //将统计好的corePoint的数目传递给一个numCorePoint
	for (int i = 0; i < numCorePoint; i++) {
		for (int j = 0; j < numCorePoint; j++) {
			double distTemp = distP2P[corePoint[i].corePointID][corePoint[j].corePointID];//获取已统计的核心点之间的距离
			if (distTemp <= Eps) {
				corePoint[i].corepts.push_back(j);  //点类得一维数组corepts用于存放核心点周围符合条件的核心点......
			}
		}
	}
	for (int i = 0; i < numCorePoint; i++) //把1个核心点周围依然小于给定半径的核心点的cluster全部统一成为1个数字
	{
		stack<point*> ps;   //ps是一个类型为point的stack指针
		if (corePoint[i].visited == 1) continue;
		clusterID++;
		corePoint[i].cluster = clusterID; //create a new cluster
		ps.push(&corePoint[i]);
		point* v;
		while (!ps.empty()) {
			v = ps.top();
			v->visited = 1;
			ps.pop();
			for (int j = 0; j < (v->corepts.size()); j++) {
				if (corePoint[v->corepts[j]].visited == 1) continue;
				corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
				corePoint[v->corepts[j]].visited = 1;
				ps.push(&corePoint[v->corepts[j]]);
			}
		}

	}

	cout << "border point,joint border point to core point" << endl;
	//border point,joint border point to core point
	for (int i = 0; i < len; i++) {
		for (int j = 0; j < numCorePoint; j++) {
			double distTemp = distP2P[i][corePoint[j].corePointID];
			if (distTemp <= Eps) {
				dataset[i].pointType = pointType_BORDER;
				dataset[i].cluster = corePoint[j].cluster;
				break;
			}
		}
	}
	/*
	cout << "output" << endl;
	//output
	//display
	for (int i = 0; i < len; i++) {
		cout << "第"; printf("%3d", (i + 1)); cout << "个数据: ";
		printf("%lf", dataset[i].x); cout << "，"; printf("%lf", dataset[i].y); cout << "，"; printf("%2d", dataset[i].cluster); cout << endl;
	}
	//save in .txt format named clustering.txt
	fstream clustering;
	clustering.open("test_result.txt", ios::out);
	for (int i = 0; i < len; i++) {
		clustering << dataset[i].x << "," << dataset[i].y << "," << dataset[i].cluster << "," << dataset[i].carnumber << "\n";
	}
	clustering.close();
	*/
	return dataset;
}


int findMaxCluster(vector<point> handledata) //找出有多少种cluster
{
	int MaxCluster = 0;
	int len = handledata.size();//数据长度
	for (int i = 0; i < len; i++)//参数初始化
	{
		if (handledata[i].cluster > MaxCluster)
			MaxCluster = handledata[i].cluster;
	}
	if (MaxCluster <= 1)
	{
		cout << "由于半径和最小车辆数参数设置问题，无法分簇或只有一个簇，请调整参数。";
		exit(100); //exit函数：中断程序的执行，返回退出代码，回到C++窗口。其中退出代码status是整型常量，返回操作系统，C++看不到exit的返回值。需要包含系统头文件stdlib.h声明使用。
	}
	else
	return MaxCluster;
}

ListNode* CreatFirstNode(ListNode* head, vector<point> handledata, int times)  //创建第一个Node
{
	for (int i = 0; i < handledata.size(); i++)
	{
		if (handledata[i].cluster == 1)
			head->elements.push_back(handledata[i]);

	}
	head->time.push_back(times);
	head->next = NULL;
	return head;
}



ListNode* CreatList(ListNode* head, int MaxCluster, vector<point> handledata, int times)  //创建第二个到最后一个链表。
{

	ListNode* p = head;
	for (int i = 2; i <= MaxCluster; i++)
	{
		ListNode* NewNode = new ListNode;
		for (int j = 0; j < handledata.size(); j++)
		{
			if (handledata[j].cluster == i)
			{
				NewNode->elements.push_back(handledata[j]);
			}
		}
		NewNode->time.push_back(times);
		NewNode->next = NULL;
		p->next = NewNode;
		p = NewNode;
	}
	return head;
}

void ShowList(ListNode* head)
{
	cout << "所属簇类      " << "车牌号     " << "时间" << endl;;
	ListNode* p = head;
	while (p)
	{
		int len = p->elements.size();
		for (int i = 0; i < len; i++)
		{
			//printf("%lf", dataset[i].x); cout << "，"; printf("%lf", dataset[i].y); cout << "，"; printf("%2d", dataset[i].cluster); cout << endl;
			//cout << p->elements[i].cluster << "        " << p->elements[i].carnumber << endl;
			printf("%3d", p->elements[i].cluster);
			cout << "         ";
			printf("%8d", int(p->elements[i].carnumber));
			cout << "      ";
			int len2 = p->time.size();
			for (int j = 0; j < len2; j++)
			{
				cout << p->time[j];
				if (j != len2 - 1)
				{
					cout << ",";
				}
			}
			cout << endl;
		}
		p = p->next;
	}
}

int FindListLength(ListNode* head)  //找出链表长度
{
	int length = 0;
	ListNode* p = head;
	while (p)
	{
		length++;
		p = p->next;
	}
	return length;
}
int FindPointInList_Length(ListNode* list)  //找出链表中每个链子里面point的个数
{
	ListNode* p = list;
	int length = p->elements.size();
	return length;
}

int FindCommonNumsInTwoListElements(ListNode* list1, ListNode* list2) //找出两个链表中任意两个子链共同拥有的车号的数量
{
	int length1 = FindPointInList_Length(list1); int length2 = FindPointInList_Length(list2);
	int count = 0;
	for (int i = 0; i < length1; i++)
	{
		for (int j = 0; j < length2; j++)
		{
			if ((list1->elements[i].carnumber) == (list2->elements[j].carnumber))
				count++;
		}
	}
	return count;
}

bool JusticeIfElementsInFirstListBelongToSecondList(ListNode* list1, ListNode* list2)  //验证链子1中point都属于链子2中的point
{
	int length1 = FindPointInList_Length(list1); int length2 = FindPointInList_Length(list2);
	if (length1 > length2)
		return false;
	else {
		int i = 0; int j = 0;
		for (j = 0; j < length2; j++)
		{
			for (i = 0; i < length1; i++)
			{
				if ((list2->elements[j].carnumber) == (list1->elements[i].carnumber))
					break;


				if (i == length1)
					return false;
			}
		}
		return true;
	}
}

ListNode* FindCommonPointsAndBothTimesInTwoListElements(ListNode* head, ListNode* list1, ListNode* list2, int times)  //取两个list的elements交集和time的并集，把这个赋值给一个新的list单元。list1必须是原始链表
{
	int alllength = FindListLength(head);//找出链表的总长度
	int length1 = FindPointInList_Length(list1); int length2 = FindPointInList_Length(list2); // 找出两个list的point数目
	ListNode* copypoint = new ListNode;  //创建一个v'，即一个新的链子
	//给v'放进错误簇数的elements
	for (int i = 0; i < length1; i++)
	{
		for (int j = 0; j < length2; j++)
		{
			if ((list1->elements[i].carnumber) == (list2->elements[j].carnumber))
			{

				copypoint->next = NULL;
				copypoint->elements.push_back(list1->elements[i]);
			}
		}
	}
	//给v'放进time
	int timelength = list1->time.size();
	for (int k = 0; k < timelength; k++)
	{
		copypoint->time.push_back(list1->time[k]);
	}
	copypoint->time.push_back(times);
	//修改copypoint->elements中point的簇
	int copypointlength = FindPointInList_Length(copypoint);
	for (int l = 0; l < copypointlength; l++)
	{
		copypoint->elements[l].cluster = alllength + 1;
	}
	return copypoint;
}

bool JusticeIfELementsInTwoListAreSame(ListNode* list1, ListNode* list2)   //判断两个链子的elements是否是完全相同的
{
	int length1 = FindPointInList_Length(list1); int length2 = FindPointInList_Length(list2); // 找出两个list的point数目
	if (length1 != length2)
	{
		return false;
	}
	else
	{
		vector<double> carnumber1(length1); vector<double>carnumber2(length2);
		for (int i = 0; i < length1; i++)
		{
			carnumber1[i] = list1->elements[i].carnumber;
		}
		for (int j = 0; j < length2; j++)
		{
			carnumber2[j] = list2->elements[j].carnumber;
		}
		sort(carnumber1.begin(), carnumber1.end());
		sort(carnumber2.begin(), carnumber2.end());
		for (int i = 0; i < length1; i++)
		{
			if (carnumber1[i] != carnumber2[i])
			{
				return false;
			}
		}
		return true;
	}
}

vector<int> GetAllTimeInTwoList(ListNode* newnode, ListNode* diferencelist) //获取两个子链时间并集
{
	int length1 = newnode->time.size(); int length2 = diferencelist->time.size();
	set<int> A;
	set<int> B;
	set<int> C;
	set<int>::iterator pos;/// 定义迭代器，作用是输出set元素
	for (int i = 0; i < length1; i++)
	{
		A.insert(newnode->time[i]);
	}
	for (int j = 0; j < length2; j++)
	{
		B.insert(diferencelist->time[j]);
	}
	set_union(A.begin(), A.end(), B.begin(), B.end(), inserter(C, C.begin()));    /*取并集运算*/
	vector<int> result;
	for (pos = C.begin(); pos != C.end(); pos++)
	{
		result.push_back(*pos);
	}
	return result;
}


void InsertCanList(ListNode* newnode, ListNode* nowlist, int length)  //判断是否插入newnode。直接对nowlist进行修改，length为原始链表长度。
{
	bool exist = false;
	int nowlistlength = FindListLength(nowlist);
	ListNode* diferencelist = NULL;
	ListNode* copynowlist = nowlist;
	ListNode* copycopynowlist = nowlist;
	ListNode* finalnowlist = nowlist;

	for (int i = 0; i < length; i++)
	{
		if (JusticeIfELementsInTwoListAreSame(newnode, copynowlist))
		{
			exist = true;
			break;
		}
		copynowlist = copynowlist->next;
	}
	if (exist == false)
	{
		if (length != nowlistlength)
		{
			for (int k = 0; k < length; k++)//把copycopynowlist指针跳到▲List的第一个
			{
				copycopynowlist = copycopynowlist->next;

			}
			diferencelist = copycopynowlist;


			for (int j = length; j < nowlistlength; j++)
			{

				if (JusticeIfELementsInTwoListAreSame(newnode, diferencelist))
				{

					vector<int> timeresult = GetAllTimeInTwoList(newnode, diferencelist);
					int timeresultlength = timeresult.size();
					diferencelist->time.clear();
					for (int i = 0; i < timeresultlength; i++)
					{
						diferencelist->time.push_back(timeresult[i]);
					}
					exist = true;
					break;
				}
				diferencelist = diferencelist->next;
			}
		}
	}

	if (exist == false)
	{
		for (int l = 0; l < (nowlistlength - 1); l++)
		{
			finalnowlist = finalnowlist->next;
		}
		finalnowlist->next = newnode;    //添加已经做完的newnode


	}
}






ListNode* GroupMode(ListNode* originalhead, ListNode* freshhead, int times) //组模式函数
{
	ListNode* list = originalhead; ListNode* newlist = freshhead;
	ListNode* finalnewlist = freshhead;

	int length = FindListLength(originalhead);   //获取原始链表总长度,这个值是不会变的在每次的运算中是不会变的。


	for (int i = 0; i < length; ++i)
	{

		for (newlist = freshhead; (newlist != NULL); newlist = newlist->next)
		{
			if (FindCommonNumsInTwoListElements(list, newlist) >= minO)
			{
				if (JusticeIfElementsInFirstListBelongToSecondList(list, newlist))
				{
					list->time.push_back(times);
					break;
				}
				else
				{
					ListNode* newnode = new ListNode;
					newnode = FindCommonPointsAndBothTimesInTwoListElements(originalhead, list, newlist, times);
					//int test10 = FindListLength(newnode);
					InsertCanList(newnode, originalhead, length);
				}
			}
		}
		list = list->next;

	}
	for (finalnewlist; (finalnewlist != NULL); finalnewlist = finalnewlist->next)
	{
		int newlength = FindListLength(originalhead);  //获取新链表长度
		int clusterlength = FindPointInList_Length(finalnewlist);
		if (clusterlength >= minO)
		{
			for (int i = 0; i < clusterlength; i++)
			{
				finalnewlist->elements[i].cluster = (newlength + 1);
			}

			ListNode* passlist = new ListNode;
			passlist->elements = finalnewlist->elements;
			passlist->time = finalnewlist->time;
			passlist->next = NULL;


			InsertCanList(passlist, originalhead, length);
		}
	}
	return originalhead;
}

void ShowAndWriteResult(ListNode* result)
{
	cout << "output" << endl;
	ListNode* p = result;
	int lengthlist = FindListLength(result);

	//output
	//display  
	/*
	for (int i = 0; i < len; i++) {
		cout << "第"; printf("%3d", (i + 1)); cout << "个数据: ";
		printf("%lf", dataset[i].x); cout << "，"; printf("%lf", dataset[i].y); cout << "，"; printf("%2d", dataset[i].cluster); cout << endl;
	}
	*/
	//save in .txt format named clustering.txt
	fstream clustering;
	clustering.open("result.txt", ios::out);
	for (p; p != NULL; p = p->next)
	{

		int pointsinelement = FindPointInList_Length(p);
		int timesnum = p->time.size();
		for (int i = 0; i < pointsinelement; i++)
		{
			clustering << p->elements[i].carnumber << ",      " << p->elements[i].cluster << ",      ";
			for (int j = 0; j < timesnum; j++)
			{
				clustering << p->time[j];
				if (j != timesnum - 1)
				{
					clustering << ",";
				}
			}
			clustering << "\n";

		}
	}
	clustering.close();

}

void ShowNumsOfList(ListNode* result)
{
	int AllCluster = 0;
	ListNode* p = result;
	
	for (p; p != NULL; p = p->next)
	{
		int pointsinelement = FindPointInList_Length(p);
		for (int i = 0; i < pointsinelement; i++)
		{
			if (p->elements[i].cluster>=AllCluster)
			{
				AllCluster = p->elements[i].cluster;
			}
		}
	}
	cout << "这次数据分得的簇有" << AllCluster << "个" << endl;

}
