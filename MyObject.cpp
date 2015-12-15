#include "MyObject.h"


MyObject::MyObject(void)
{
	SavedImageCounter=0;
}


MyObject::~MyObject(void)
{
	//Images.~list();
	//matcher.~FlannBasedMatcher();
	//int o=2;
}
void MyObject::AddImage(Mat&I)
{
	Images.push_back(I);
}
void MyObject::SaveImages(string ParentDirPath,int ObjectNumber)
{
	
	char buffer[500];
	/*
	char buffer2[500];
	
	sprintf(buffer,"%s\\%d",ParentDirPath.c_str(),ObjectNumber);
	//check on folder to exist or create it
	if(_access(buffer,0)!=0)//if the folder doesn't exist
	{
		sprintf(buffer2,"mkdir %s",buffer);
		system(buffer2);//creating a directory using batch command
	}
	*/
	if(mrpt::system::directoryExists(string(buffer))==false)//create the directory if it doesn't exist
		mrpt::system::createDirectory(string(buffer));
	for(list<Mat>::iterator i=Images.begin();i!=Images.end();i++)
	{
		sprintf(buffer,"%s/%d_%d.png",ParentDirPath.c_str(),ObjectNumber,SavedImageCounter);
		imwrite(buffer,*i);
		SavedImageCounter++;
	}
	Images.clear();//clear the list after the images are saved to disk
}
void MyObject::SaveMatcher(string MatcherDirPath,int ObjectNumber)
{
	char buffer[500];

	if(mrpt::system::directoryExists(string(buffer))==false)//create the directory if it doesn't exist
		mrpt::system::createDirectory(string(buffer));

	sprintf(buffer,"%s/%d_descriptor.yaml",MatcherDirPath.c_str(),ObjectNumber);
	FileStorage f(buffer,FileStorage::WRITE);//will it make error if file doeen'y exist or it will just create it
	
	vector<Mat> tmp=matcher.getTrainDescriptors();
	char tmpbuffer[10];

	//write number of descriptors matrices=number of images of that object
	f<<"N"<<(int)tmp.size();//conversion to int due to ambiguius call to <<

	for(int i=0;i<tmp.size();i++)
	{
		//sprintf(tmpbuffer,"%d",i);
		//f.writeObj(tmpbuffer,&(tmp.at(i)));
		//f<<i<<tmp.at(i);
		sprintf(tmpbuffer,"M%d",i);
		f<<tmpbuffer<<tmp.at(i);
	}
	f.release();
}
void MyObject::AddDescriptor(Mat&D)
{
	vector<Mat> tmp(1);//=new vector<Mat>(1);
	tmp.at(0)=D;
	matcher.add(tmp);
}
float MyObject::GetAvgDescriptorDistance(Mat&QueryDescriptor)
{
	vector<DMatch> distances;
	//for debugging
	//FileStorage f("Descriptor.yaml",FileStorage::WRITE);//
	//matcher.write(f);
	//f.release();
	//end debugging commands
	matcher.match(QueryDescriptor,distances);//there was a bug here but it was fixed , it was a result of calling train then match, if match was called only then there is no bug
	int n=distances.size();
	float sum=0;
	for(int i=0;i<n;i++)
		sum+=distances.at(i).distance;
	return sum/n;
}