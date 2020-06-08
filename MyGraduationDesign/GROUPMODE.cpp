#include "GROUPMODE.h"

const int minO = 5;
const double PIx = 3.141592653589793;
const double RADIUS = 6378.16;

void point::init()     //��ʼ��point�����е�int����
{
	cluster = 0;
	pointType = pointType_UNDO;//pointType_NOISE pointType_UNDO
	pts = 0;                   //points in MinPts 
	visited = 0;
	corePointID = -1;
}


double stringToDouble(string i)   //ͨ��ʹ��stringstreamʹ��string���͵�i����ת��Ϊfloat����
{
	stringstream sf;    //#include <sstream>�����Ķ�����
	double score = 0;
	sf << i;
	sf >> score;
	return score;
}


vector<point> openFile(const char* dataset)   //��txt������double����ʽ����һ��point���У����
{
	//�ȴ��ļ�
	fstream file;
	file.open(dataset, ios::in);
	if (!file)    //������ļ�ʧ��
	{
		cout << "Open File Failed!" << endl;
		vector<point> a;
		return a;
	}
	//�����ļ�
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

double squareDistance(point a, point b)   //����������֮���ֱ�߾���
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

	int len = dataset.size();//���ݳ���
	for (int i = 0; i < len; i++)//������ʼ��
	{
		dataset[i].init();
	}


	vector<vector <double>> distP2P(len);   //����һ����ά���顣���ڷ���ÿ���㵽������ľ��롣
	//����ÿ���㵽������ľ���С�ڹ涨����Eps�ĸ�����
	cout << "calculate pts" << endl;
	for (int i = 0; i < len; i++) {
		for (int j = 0; j < len; j++)
		{//i+1
			double distance = squareDistance(dataset[i], dataset[j]); //����dataset���ÿ��point�ൽ������ľ��룻
			distP2P[i].push_back(distance); //��ÿ������Ž�һ���µĶ�ά����distP2P�С���һά���������ڶ�ά�����point�ൽ������ľ��뼯�ϡ�
			if (distance <= Eps)
			{
				dataset[i].pts++;
			}
		}
	}

	//�ҳ�corepoint������Щ����Ž�һ��һά����֮�С�
	cout << "core point " << endl;
	vector<point> corePoint;
	for (int i = 0; i < len; i++) {
		int tempPts = dataset[i].pts;  //��ÿ������Χ���������ĵ����Ž�tempPts֮��
		if (tempPts >= MinPts) {       //���������Ļ�
			dataset[i].pointType = pointType_CORE;   //������������pointType��corePointID
			dataset[i].corePointID = i;
			corePoint.push_back(dataset[i]);//���ҽ���Щ���෵��һ��corePoint���顣
		}
	}
	cout << "joint core point" << endl;

	//joint core point
	int numCorePoint = corePoint.size(); //��ͳ�ƺõ�corePoint����Ŀ���ݸ�һ��numCorePoint
	for (int i = 0; i < numCorePoint; i++) {
		for (int j = 0; j < numCorePoint; j++) {
			double distTemp = distP2P[corePoint[i].corePointID][corePoint[j].corePointID];//��ȡ��ͳ�Ƶĺ��ĵ�֮��ľ���
			if (distTemp <= Eps) {
				corePoint[i].corepts.push_back(j);  //�����һά����corepts���ڴ�ź��ĵ���Χ���������ĺ��ĵ�......
			}
		}
	}
	for (int i = 0; i < numCorePoint; i++) //��1�����ĵ���Χ��ȻС�ڸ����뾶�ĺ��ĵ��clusterȫ��ͳһ��Ϊ1������
	{
		stack<point*> ps;   //ps��һ������Ϊpoint��stackָ��
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
		cout << "��"; printf("%3d", (i + 1)); cout << "������: ";
		printf("%lf", dataset[i].x); cout << "��"; printf("%lf", dataset[i].y); cout << "��"; printf("%2d", dataset[i].cluster); cout << endl;
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


int findMaxCluster(vector<point> handledata) //�ҳ��ж�����cluster
{
	int MaxCluster = 0;
	int len = handledata.size();//���ݳ���
	for (int i = 0; i < len; i++)//������ʼ��
	{
		if (handledata[i].cluster > MaxCluster)
			MaxCluster = handledata[i].cluster;
	}
	if (MaxCluster <= 1)
	{
		cout << "���ڰ뾶����С�����������������⣬�޷��ִػ�ֻ��һ���أ������������";
		exit(100); //exit�������жϳ����ִ�У������˳����룬�ص�C++���ڡ������˳�����status�����ͳ��������ز���ϵͳ��C++������exit�ķ���ֵ����Ҫ����ϵͳͷ�ļ�stdlib.h����ʹ�á�
	}
	else
	return MaxCluster;
}

ListNode* CreatFirstNode(ListNode* head, vector<point> handledata, int times)  //������һ��Node
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



ListNode* CreatList(ListNode* head, int MaxCluster, vector<point> handledata, int times)  //�����ڶ��������һ������
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
	cout << "��������      " << "���ƺ�     " << "ʱ��" << endl;;
	ListNode* p = head;
	while (p)
	{
		int len = p->elements.size();
		for (int i = 0; i < len; i++)
		{
			//printf("%lf", dataset[i].x); cout << "��"; printf("%lf", dataset[i].y); cout << "��"; printf("%2d", dataset[i].cluster); cout << endl;
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

int FindListLength(ListNode* head)  //�ҳ�������
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
int FindPointInList_Length(ListNode* list)  //�ҳ�������ÿ����������point�ĸ���
{
	ListNode* p = list;
	int length = p->elements.size();
	return length;
}

int FindCommonNumsInTwoListElements(ListNode* list1, ListNode* list2) //�ҳ�������������������������ͬӵ�еĳ��ŵ�����
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

bool JusticeIfElementsInFirstListBelongToSecondList(ListNode* list1, ListNode* list2)  //��֤����1��point����������2�е�point
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

ListNode* FindCommonPointsAndBothTimesInTwoListElements(ListNode* head, ListNode* list1, ListNode* list2, int times)  //ȡ����list��elements������time�Ĳ������������ֵ��һ���µ�list��Ԫ��list1������ԭʼ����
{
	int alllength = FindListLength(head);//�ҳ�������ܳ���
	int length1 = FindPointInList_Length(list1); int length2 = FindPointInList_Length(list2); // �ҳ�����list��point��Ŀ
	ListNode* copypoint = new ListNode;  //����һ��v'����һ���µ�����
	//��v'�Ž����������elements
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
	//��v'�Ž�time
	int timelength = list1->time.size();
	for (int k = 0; k < timelength; k++)
	{
		copypoint->time.push_back(list1->time[k]);
	}
	copypoint->time.push_back(times);
	//�޸�copypoint->elements��point�Ĵ�
	int copypointlength = FindPointInList_Length(copypoint);
	for (int l = 0; l < copypointlength; l++)
	{
		copypoint->elements[l].cluster = alllength + 1;
	}
	return copypoint;
}

bool JusticeIfELementsInTwoListAreSame(ListNode* list1, ListNode* list2)   //�ж��������ӵ�elements�Ƿ�����ȫ��ͬ��
{
	int length1 = FindPointInList_Length(list1); int length2 = FindPointInList_Length(list2); // �ҳ�����list��point��Ŀ
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

vector<int> GetAllTimeInTwoList(ListNode* newnode, ListNode* diferencelist) //��ȡ��������ʱ�䲢��
{
	int length1 = newnode->time.size(); int length2 = diferencelist->time.size();
	set<int> A;
	set<int> B;
	set<int> C;
	set<int>::iterator pos;/// ��������������������setԪ��
	for (int i = 0; i < length1; i++)
	{
		A.insert(newnode->time[i]);
	}
	for (int j = 0; j < length2; j++)
	{
		B.insert(diferencelist->time[j]);
	}
	set_union(A.begin(), A.end(), B.begin(), B.end(), inserter(C, C.begin()));    /*ȡ��������*/
	vector<int> result;
	for (pos = C.begin(); pos != C.end(); pos++)
	{
		result.push_back(*pos);
	}
	return result;
}


void InsertCanList(ListNode* newnode, ListNode* nowlist, int length)  //�ж��Ƿ����newnode��ֱ�Ӷ�nowlist�����޸ģ�lengthΪԭʼ�����ȡ�
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
			for (int k = 0; k < length; k++)//��copycopynowlistָ��������List�ĵ�һ��
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
		finalnowlist->next = newnode;    //����Ѿ������newnode


	}
}






ListNode* GroupMode(ListNode* originalhead, ListNode* freshhead, int times) //��ģʽ����
{
	ListNode* list = originalhead; ListNode* newlist = freshhead;
	ListNode* finalnewlist = freshhead;

	int length = FindListLength(originalhead);   //��ȡԭʼ�����ܳ���,���ֵ�ǲ�������ÿ�ε��������ǲ����ġ�


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
		int newlength = FindListLength(originalhead);  //��ȡ��������
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
		cout << "��"; printf("%3d", (i + 1)); cout << "������: ";
		printf("%lf", dataset[i].x); cout << "��"; printf("%lf", dataset[i].y); cout << "��"; printf("%2d", dataset[i].cluster); cout << endl;
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
	cout << "������ݷֵõĴ���" << AllCluster << "��" << endl;

}
