#include <tinyxml2.h>
#include <iostream>
#include <vector>

using namespace std;

bool loadPathInfo(const std::string& file)
{
	using namespace tinyxml2;
	tinyxml2::XMLDocument Doc;  
	XMLError res = Doc.LoadFile(file.c_str());
	
	if(XML_ERROR_FILE_NOT_FOUND == res)
	{
		cout << "parking points file: "<< file << "not exist!" << endl;
		return false;
	}
	else if(XML_SUCCESS != res)
	{
		cout << "parking points file: "<< file << " parse error!" << endl;
		return false;
	}
	tinyxml2::XMLElement *pRoot=Doc.RootElement();//根节点
	
	tinyxml2::XMLElement *pParkingPoints=pRoot->FirstChildElement("ParkingPoints");
	
	if(pParkingPoints)
	{
		tinyxml2::XMLElement *pParkingPoint = pParkingPoints->FirstChildElement("ParkingPoint");
		while (pParkingPoint)
		{
			uint32_t id = pParkingPoint->Unsigned64Attribute("id");
			uint32_t index = pParkingPoint->Unsigned64Attribute("index");
			float duration = pParkingPoint->FloatAttribute("duration");
			cout << id << "\t" << index << "\t" << duration << endl;
			//转到下一子节点
			pParkingPoint = pParkingPoint->NextSiblingElement("ParkingPoint");  
		}
	}
	
	tinyxml2::XMLElement *pTurnRanges = pRoot->FirstChildElement("TurnRanges");
	if(pTurnRanges)
	{
		tinyxml2::XMLElement *pTurnRange = pTurnRanges->FirstChildElement("TurnRange");
		while (pTurnRange)
		{
			int id = pTurnRange->IntAttribute("type");
			uint32_t index = pTurnRange->Unsigned64Attribute("start");
			float duration = pTurnRange->FloatAttribute("end");
			cout << id << "\t" << index << "\t" << duration << endl;
			//转到下一子节点
			pTurnRange = pTurnRange->NextSiblingElement("TurnRange");  
		}
	}
	
	
}

std::vector<int > path_points_(100);

bool generatePathInfoFile(const std::string& file_name)
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

int main()
{
	loadPathInfo("test.xml");
	//generatePathInfoFile("test.xml");
	return 0;
}







