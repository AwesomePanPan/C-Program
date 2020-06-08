#if 1
#include "GROUPMODE.h"

void main()
{

	const char* textname[22] =
	{
	 "realpoint2.txt",
	 "realpoint3.txt",
	 "realpoint4.txt",
	 "realpoint5.txt",
	 "realpoint6.txt",
	 "realpoint7.txt",
	 "realpoint8.txt",
	 "realpoint9.txt",
	 "realpoint10.txt",
	 "realpoint11.txt",
	 "realpoint12.txt",
	 "realpoint13.txt",
	 "realpoint14.txt",
	 "realpoint15.txt",
	 "realpoint16.txt",
	 "realpoint17.txt",
	 "realpoint18.txt",
	 "realpoint19.txt",
	 "realpoint20.txt",
	 "realpoint21.txt",
	 "realpoint22.txt",
	 "realpoint23.txt"
	};
	const char* faketestname[3] =
	{
		"testpoint2.txt","testpoint3.txt","testpoint4.txt"
	};

	//第一个链表制作如下：
	int firsttimes = 1;
	cout << "第1组数据计算" << endl;
	vector<point> dataset = openFile("realpoint1.txt");
	dataset = DBSCAN(dataset,300, 10);
	int MaxCluster = findMaxCluster(dataset);
	ListNode* head = new ListNode;
	head = CreatFirstNode(head, dataset, firsttimes);
	head = CreatList(head, MaxCluster, dataset, firsttimes);
	ShowNumsOfList(head);
	//ShowNumsOfList(head);
	//ShowList(head);
	
	//做出后面的链表,并且处理
	ListNode* result = head;
	for (int i = 0; i <22; i++)
	{
		cout << "第" << i + 2 << "组数据计算" << endl;
		int times = i + 2;
		vector<point> dataset2 = openFile(textname[i]);
		dataset2 = DBSCAN(dataset2,300, 10);
		int MaxCluster2 = findMaxCluster(dataset2);
		ListNode* head2 = new ListNode;
		head2 = CreatFirstNode(head2, dataset2, times);
		head2 = CreatList(head2, MaxCluster2, dataset2, times);
		result = GroupMode(result, head2, times);
		ShowNumsOfList(result);
	}
	ShowAndWriteResult(result);
}

#endif