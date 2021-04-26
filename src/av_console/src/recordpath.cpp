#include "../include/recordpath.hpp"

RecordPath::RecordPath():
    odom_topic_("/ll2utm_odom"),
    row_num_(0)
{

}

RecordPath::~RecordPath()
{
    path_points_.clear();
}

//若不存在发布者，返回错误，外部自主启动发布者
bool RecordPath::start()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<float>("sample_distance",sample_distance_,0.1);
    sub_gps_ = nh.subscribe(odom_topic_ ,1,&RecordPath::gps_callback,this);
    connect(&wait_gps_topic_timer_, SIGNAL(timeout()), this, SLOT(waitGpsTopicTimeout()));

    path_points_.reserve(5000);
    ros::Duration(0.5).sleep();  //等待订阅器初始化完成，否则即使存在发布者，也有可能被漏检

    if(sub_gps_.getNumPublishers() == 0) //检测发布者是否存在
        return false;
    return true;
}

void RecordPath::waitGpsTopicTimeout()
{
    if(sub_gps_.getNumPublishers() == 0)
    {

    }
}

void RecordPath::stop()
{
    sub_gps_.shutdown();
}

void RecordPath::gps_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gpsPoint current_point;
  current_point.x = msg->pose.pose.position.x;
  current_point.y = msg->pose.pose.position.y;
  current_point.yaw = msg->pose.covariance[0];

  if(sample_distance_*sample_distance_ <= dis2Points(current_point,last_point_,false))
  {
    path_points_.push_back(current_point);
    //fprintf(fp,"%.3f\t%.3f\t%.4f\n",current_point.x,current_point.y,current_point.yaw);
    last_point_ = current_point;

    std::stringstream msg;
    msg << ++row_num_ << "\t" << std::fixed << std::setprecision(2)
        << current_point.x << "\t"
        << current_point.y << "\t"
        << current_point.yaw*180.0/M_PI;
    log("INFO",msg.str());
  }
}

bool RecordPath::savePathPoints(const std::string& file_name)
{
   std::ofstream out_file;
   out_file.open(file_name.c_str());

   if(!out_file.is_open())
   {
     std::stringstream ss;
     ss << "open " <<  file_name << " failed!";
     log("ERROR",ss.str());
     return false;
   }

   for(size_t i=0; i<path_points_.size(); ++i)
   {
     out_file << std::fixed << std::setprecision(2)
              << path_points_[i].x << "\t" << path_points_[i].y << "\t"
              << path_points_[i].yaw << "\r\n";
   }
   out_file.close();

   std::stringstream ss;
   ss << "path points saved in " << file_name;
   this->log("INFO",ss.str());

   // generate curvature
   FILE * fp =  popen("rospack find av_console", "r");
   char buf[50] ;
   fscanf(fp,"%s",buf);
   pclose(fp);
   QDir cmdDir = QDir::current();//获取当前工作目录
   cmdDir.cd(QString(buf));      //修改目录，仅修改了目录名，未切换
   cmdDir.cd("scripts");
   QDir::setCurrent(cmdDir.absolutePath()); //切换目录

   std::string tool_file = "generate_curvature.py";
   std::string cmd = std::string("python ") + tool_file + " " + file_name ;
   std::cout << "\n=====================================================\n";
   std::cout << "start to generate curvature... "<< std::endl;
   system(cmd.c_str());
   std::cout << "generate curvature complete..." << std::endl;
   std::cout << "=====================================================\n";

   return true;
}

float RecordPath::dis2Points(gpsPoint& p1,gpsPoint&p2,bool isSqrt)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  if(isSqrt)
    return sqrt(dx*dx+dy*dy);
  return dx*dx+dy*dy;
}

void RecordPath::log( const std::string &level, const std::string &msg)
{
  if(logging_model.rowCount() >400)
  {
      logging_model.removeRows(0,200);
  }

  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;

  logging_model_msg << std::fixed << std::setprecision(3)
                    << "[" << level << "]" << "[" << ros::Time::now() << "]: " << msg;

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

/*@brief 生成路径信息文件
 *1. parking points
 *2. turn
 *@param file_name 文件
 */
#include<tinyxml2.h>
bool RecordPath::generatePathInfoFile(const std::string& file_name)
{
    tinyxml2::XMLDocument doc;
    //1.添加声明
    tinyxml2::XMLDeclaration* declaration = doc.NewDeclaration();
    doc.InsertFirstChild(declaration); //在最前插入声明

    //2.创建根节点
    tinyxml2::XMLElement* pathInfoNode = doc.NewElement("PathInfo");
    doc.InsertEndChild(pathInfoNode);  //在最后插入根节点

    { //ParkingPoints
    tinyxml2::XMLElement* parkingPointsNode = doc.NewElement("ParkingPoints");
    pathInfoNode->InsertEndChild(parkingPointsNode);

    // 创建Description子节点,并插入父节点
    tinyxml2::XMLElement* discriptionNode = doc.NewElement("Description");
    parkingPointsNode->InsertEndChild(discriptionNode);

    tinyxml2::XMLElement* idElement = doc.NewElement("id");
    discriptionNode->InsertEndChild(idElement);
    idElement->InsertEndChild(doc.NewText("the sequence of the parking point"));

    tinyxml2::XMLElement* indexElement = doc.NewElement("index");
    discriptionNode->InsertEndChild(indexElement);
    indexElement->InsertEndChild(doc.NewText("the parking point position in global path"));

    tinyxml2::XMLElement* durationElement = doc.NewElement("duration");
    discriptionNode->InsertEndChild(durationElement);
    durationElement->InsertEndChild(doc.NewText("parking time(s), 0 for destination"));

    tinyxml2::XMLElement* addEle = doc.NewElement("add");
    discriptionNode->InsertEndChild(addEle);
    addEle->InsertEndChild(doc.NewText("To add a parking point manually, please follow the format below"));

    //创建ParkingPoint节点
    tinyxml2::XMLElement* pointElement = doc.NewElement("ParkingPoint");
    parkingPointsNode->InsertEndChild(pointElement); //在最后插入节点

    //为节点增加属性
    pointElement->SetAttribute("id", 0);
    pointElement->SetAttribute("index", path_points_.size()-1);
    pointElement->SetAttribute("duration", 0);
    }

    {//TurnRanges
    tinyxml2::XMLElement* turnRangesNode = doc.NewElement("TurnRanges");
    pathInfoNode->InsertEndChild(turnRangesNode);

    //创建Description子节点,并插入父节点
    tinyxml2::XMLElement* discriptionNode = doc.NewElement("Description");
    turnRangesNode->InsertEndChild(discriptionNode);

    tinyxml2::XMLElement* typeElement = doc.NewElement("type");
    discriptionNode->InsertEndChild(typeElement);
    typeElement->InsertEndChild(doc.NewText("-1: left turn, 0: none, 1: right turn"));

    tinyxml2::XMLElement* startElement = doc.NewElement("start");
    discriptionNode->InsertEndChild(startElement);
    startElement->InsertEndChild(doc.NewText("the start index of turn"));

    tinyxml2::XMLElement* endElement = doc.NewElement("end");
    discriptionNode->InsertEndChild(endElement);
    endElement->InsertEndChild(doc.NewText("the end index of turn"));

    tinyxml2::XMLElement* addEle = doc.NewElement("add");
    discriptionNode->InsertEndChild(addEle);
    addEle->InsertEndChild(doc.NewText("To add a turn range manually, please follow the format below"));

    //创建TurnRange节点
    tinyxml2::XMLElement* turnRangeNode = doc.NewElement("TurnRange");
    turnRangesNode->InsertEndChild(turnRangeNode);

    //添加属性,起步左转
    turnRangeNode->SetAttribute("type", -1);
    turnRangeNode->SetAttribute("start", 0);
    turnRangeNode->SetAttribute("end", 50);
    }
    //6.保存xml文件
    doc.SaveFile(file_name.c_str());
}
