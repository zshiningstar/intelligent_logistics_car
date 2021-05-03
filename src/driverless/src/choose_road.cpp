##  编写一个cpp文件开始就把所有的路径读入,存放在对应的vector里面

仓库b2
	包含进入b2,退出b2,前往b7三段路径
仓库b7
	包含进入b7,退出b7,前往b10三段路径
仓库b10
	包含进入b10,退出b10,前往b2三段路径

bool isCurrentRoadEnd		///当前记录路径结束


//判断当前是在室内还是室外可通过slam-gps主驾驶线路得出
//当前在哪个仓库也可以通过slam得出
//假如当前为进入b2仓库
//	bool is_road_forward		//道路编号(正向进入仓库)   
//	bool is_road_backward 		//道路编号(反向倒车离开仓库)
//	bool is_road_across				//从一个擦仓库到另外一个仓库
//is_road_forward置为true,is_road_backward,is_road_across为false

std::vector<bool> RoadIndex[3];
RoadIndex[0] = true;
RoadIndex[1] = false;
RoadIndex[2] = false;



uint8_t warehouseIndex;		//仓库编号
RoadIndex_t roadIndex;

if as_->setSucceeded(driverless::DoDriverlessTaskResult(), "drive work  completed");  //当前路段自动驾驶完成
	isCurrentRoadEnd = true;
	
if(isCurrentRoadEnd)
{
	switch(warehouseIndex)
	{
		case 'b2' :
			if(RoadIndex[0])
			{
				RoadIndex[0] = false;
				RoadIndex[1] = true;
				RoadIndex[2] = false;
			}
			else if(RoadIndex[1])
			{
				RoadIndex[1] = false;
				RoadIndex[2] = true;
				RoadIndex[0] = false;
			}
			else if(RoadIndex[2])
			{
				RoadIndex[2] = false;
				RoadIndex[0] = true;
				RoadIndex[1] = false;
			}
			break;
		case 'b7' :
		case 'b5' :
	}//至此已经判断出此时在哪个仓库,哪个路段结束
}

另开一个线程不停的检测RoadIndex中三个的数值; **但是前提是isCurrentRoadEnd为true才进行检测**
if(RoadIndex[0])
	载入is_road_forward进入仓库的路段
else if(RoadIndex[1])
	载入退出仓库的路段
else if(RoadIndex[2])
	载入前往另外一个仓库的路段

