#include "PathDirector.h"


PathDirector::PathDirector()
{
	map=NULL;
	DTarray=NULL;
}


PathDirector::~PathDirector(void)
{
	//Delete Map and DtArray
	if(map!=NULL)
	{
		for (int i=0;i<xmax;i++)
			{
				delete [] map[i];		
			}
			delete [] map;
	}

	if(map!=NULL)
	{
		for (int i=0;i<xmax;i++)
			{
				delete [] DTarray[i];		
			}
			delete [] DTarray;
	}
}


void PathDirector::InitializeInterfaces()//Interfaces between my code and the path planning code
{
			xmax=GridMap->getSizeX();
			ymax=GridMap->getSizeY();
			
			//building map
			map=new int*[xmax];
			for (int i=0;i<xmax;i++)
			{
				map[i]=new int[ymax];
			}
			for (int i=0;i<xmax;i++)
			{
				for (int j=0;j<ymax;j++)
					{
						if(GridMap->getCell(i,j)>0.5)
						{
							map[i][j]=0;
						}
						else
						{
							map[i][j]=1;
						}
					}
			}

			InitializationValue=mrpt::math::max(xmax,ymax)+50;//calculation of Initialization Value

			//allocating DTarray
			DTarray=new int*[xmax];
			for (int i=0;i<xmax;i++)
			{
				DTarray[i]=new int[ymax];
			}

}

void PathDirector::forwardscan(int ** cspace,int xg,int yg)
{
	for(int i=0;i<xmax;i++)
	{
		for(int j=0;j<ymax;j++)
		{
			if(cspace[i][j] != 0)
			{
				//eight connectivity
				int r[2] = {i,j+1};
				int l[2] = {i,j-1};
				int u[2] = {i-1,j};
				int d[2] = {i+1,j};
				int ur[2] = {i-1,j+1};
				int ul[2] = {i-1,j-1};
				int dr[2] = {i+1,j+1};
				int dl[2] = {i+1,j-1};
				double R,L,U,D,UL,UR,DL,DR;
				double min;

				if(i==0 && j==0)
				{
					
					D = DTarray[d[0]][d[1]] + 1;
					if(D < DTarray[i][j])
						DTarray[i][j] = D;
				}
				else if( i == 0 && j==9)//
				{
					L = DTarray[l[0]][l[1]] + 1;
					D = DTarray[d[0]][d[1]] + 1;
					DL = DTarray[dl[0]][dl[1]] + 1.414;
					if(L < D)
					{
						if(L<DL)
							min = L;
						else
							min = DL;
					}
					else
					{
						if(D<DL)
							min = D;
						else
							min = DL;
					}
					if(min < DTarray[i][j])
						DTarray[i][j] = min;
				}
				else if(i==(xmax-1) && j==0)
				{
				}
				else if(i==(xmax-1) && j==(ymax-1))
				{
					L = DTarray[l[0]][l[1]] + 1;
					if(L < DTarray[i][j])
						DTarray[i][j] = L;
				}
				else
				{
					if(i == (xmax-1))
					{
						L = DTarray[l[0]][l[1]] + 1;
						if(L < DTarray[i][j])
							DTarray[i][j] = L;
					}
					else if(j == 0)
					{
						D = DTarray[d[0]][d[1]] + 1;
						if(D < DTarray[i][j])
							DTarray[i][j] = D;
					}
					else if(j == (ymax-1))
					{
						D = DTarray[d[0]][d[1]] + 1;
						L = DTarray[l[0]][l[1]] + 1;
						DL = DTarray[dl[0]][dl[1]] + 1.414;
						if(D < L)
						{
							if(D<DL)
								min = D;
							else
								min = DL;
						}
						else
						{
							if(L<DL)
								min = L;
							else
								min = DL;
						}
						if(min < DTarray[i][j])
							DTarray[i][j] = min;
					}
					else
					{
						D = DTarray[d[0]][d[1]] + 1;
						L = DTarray[l[0]][l[1]] + 1;
						DL = DTarray[dl[0]][dl[1]] + 1.414;
						DR = DTarray[dr[0]][dr[1]] + 1.414;
						if(D < L)
						{
							if(D<DL)
							{
								if(D<DR)
									min = D;
								else
									min = DR;
							}
							else
							{
								if(DL<DR)
									min = DL;
								else
									min = DR; 
							}
						}
						else
						{
							if(L<DL)
							{
								if(L<DR)
									min = L;
								else
									min = DR;
							}
							else
							{
								if(DL<DR)
									min = DL;
								else
									min = DR; 
							}
						}
						if(min < DTarray[i][j])
							DTarray[i][j] = min;
					}
				}

			}
			else
				DTarray[i][j] = 999;
		}
	}
}


void PathDirector::initialize(int xg,int yg)
{
	for(int i=0;i<xmax;i++)
	{
		for(int j=0;j<ymax;j++)
		{
			if(i == xg && j == yg)
				DTarray[i][j] = InitializationValue;
			else
				DTarray[i][j] = InitializationValue;
		}
	}
}

void PathDirector::reversescan(int ** cspace,int xg,int yg)
{
	for(int i=(xmax-1);i>=0;i--)
	{
		for(int j=(ymax-1);j>=0;j--)
		{
			if(cspace[i][j] != 0)
			{
				//eight connectivity
				int r[2] = {i,j+1};
				int l[2] = {i,j-1};
				int u[2] = {i-1,j};
				int d[2] = {i+1,j};
				int ur[2] = {i-1,j+1};
				int ul[2] = {i-1,j-1};
				int dr[2] = {i+1,j+1};
				int dl[2] = {i+1,j-1};
				double R,L,U,D,UL,UR,DL,DR;
				double min;

				if(i==0 && j==0)
				{
					
					R = DTarray[r[0]][r[1]] + 1;
					if(R < DTarray[i][j])
						DTarray[i][j] = R;
				}
				else if(i==0 && j==(ymax-1))
				{
				}
				else if(i==(xmax-1) && j==0)
				{
					U = DTarray[u[0]][u[1]] + 1;
					R = DTarray[r[0]][r[1]] + 1;
					UR = DTarray[ur[0]][ur[1]] + 1.414;
					if(U<R)
					{
						if(U<UR)
							min = U;
						else
							min = UR;
					}
					else
					{
						if(R<UR)
							min = R;
						else
							min = UR; 
					}
					if(min < DTarray[i][j])
						DTarray[i][j] = min;
				}
				else if(i==(xmax-1) && j==(ymax-1))
				{
					U = DTarray[u[0]][u[1]] + 1;
					UL = DTarray[ul[0]][ul[1]] + 1.414;
					if(U<UL)
						min = U;
					else
						min = UL;
					if(min < DTarray[i][j])
						DTarray[i][j] = min;	
				}
				else
				{
					if(i == 0)
					{
						
						R = DTarray[r[0]][r[1]] + 1;
						if(R < DTarray[i][j])
							DTarray[i][j] = R;
					}
					else if(i == (xmax-1))
					{
						U = DTarray[u[0]][u[1]] + 1;
						UL = DTarray[ul[0]][ul[1]] + 1.414;
						UR = DTarray[ur[0]][ur[1]] + 1.414;
						if(U<UR)
						{
							if(U<UL)
								min = U;
							else
								min = UL;
						}
						else
						{
							if(UR<UL)
								min = UR;
							else
								min = UL; 
					}
					if(min < DTarray[i][j])
						DTarray[i][j] = min;
					}
					else if(j == 0)
					{
						U = DTarray[u[0]][u[1]] + 1;
						R = DTarray[r[0]][r[1]] + 1;
						UR = DTarray[ur[0]][ur[1]] + 1.414;
						if(U<R)
						{
							if(U<UR)
								min = U;
							else
								min = UR;
						}
						else
						{
							if(R<UR)
								min = R;
							else
								min = UR; 
						}
						if(min < DTarray[i][j])
							DTarray[i][j] = min;
					}
					else if(j == (ymax-1))
					{
						U = DTarray[u[0]][u[1]] + 1;
						UL = DTarray[ul[0]][ul[1]] + 1.414;
					    if(U<UL)
							min = U;
						else
							min = UL;
				
						if(min < DTarray[i][j])
							DTarray[i][j] = min;
					}
					else
					{
						U = DTarray[u[0]][u[1]] + 1;
						R = DTarray[r[0]][r[1]] + 1;
						UL = DTarray[ul[0]][ul[1]] + 1.414;
						UR = DTarray[ur[0]][ur[1]] + 1.414;
						if(U<R)
						{
							if(U<UR)
							{
								if(U<UL)
									min = U;
								else
									min = UL;
							}
							else
							{
								if(UR<UL)
									min = UR;
								else
									min = UL;
							}
						}
						else
						{
							if(R<UR)
							{
								if(R<UL)
									min = R;
								else
									min = UL;
							}
							else
							{
								if(UR <UL)
									min = UR;
								else
									min = UL;
							}
						}
						if(min < DTarray[i][j])
							DTarray[i][j] = min;
					}
				}

			}
			else
				DTarray[i][j] = 999;
		}
	}
}

char PathDirector::DTPP(int xstart,int ystart,int xgoal,int ygoal)
{
	double U,D,R,L;
	double theta = 90;
	char Buffer;
	//S->Read(&Buffer,1);
	 while(xstart != xgoal || ystart != ygoal)
	 {
		 DTarray[xstart][ystart]+=5;
		if(xstart == 0 && ystart == 0)
		{
			D = DTarray[xstart+1][ystart];
		    R = DTarray[xstart][ystart+1];
			if(D<R)
			{
				xstart++;
				if(theta == 90)
				{
				    //rotate 180c and move one step
					Buffer='d';
					//S->WriteBuffer(&Buffer,1);
				}
				else if(theta == 0)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
				}
				else if(theta == 180)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = -90;
			}
			else
			{
				ystart++;
				if(theta == 90)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
				}
				else if(theta == 180)
				{
				//rotate 180c and move one step
					Buffer='d';
					//S->WriteBuffer(&Buffer,1);
				}
				else if(theta == -90)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 0;
			}
			return Buffer;
		}
	   else if(xstart == 0 && ystart == (ymax-1))
	   {
		   D = DTarray[xstart+1][ystart];
		   L = DTarray[xstart][ystart-1];
		   if(D<L)
		   {
			   xstart++;
			   if(theta == 90)
			   {
				//rotate 180c and move one step
				   Buffer='d';
				   //S->WriteBuffer(&Buffer,1);
			   }
			   else if(theta == 180)
			   {
				//rotate 90a and move one step
				   Buffer='a';
					//S->WriteBuffer(&Buffer,1);
			   }
			   else if(theta == 0)
			   {
				//rotate 90c and move one step
				   Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			   }
				theta = -90;
		   }
		   else
		   {
			   ystart--;
			   if(theta == 90)
			   {
				//rotate 90a and move one step
				   Buffer='a';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == -90)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 0)
				{
				//rotate 180c and move one step
					Buffer='d';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 180;
		   }
		   return Buffer;
	   }
	   else if(xstart == (xmax-1) && ystart == 0)
	   {
		   U = DTarray[xstart-1][ystart];
		   R = DTarray[xstart][ystart+1];
		   if(U<R)
		   {
			   xstart--;
			   if(theta == -90)
			   {
				//rotate 180c and move one step
				   Buffer='d';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 180)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 0)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 90;
		   }
		   else
		   {
			   ystart++;
			   if(theta == 90)
			   {
				//rotate 90c and move one step
				   Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 180)
				{
				//rotate 180c and move one step
					Buffer='d';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == -90)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 0;
		   }
		   return Buffer;
	   }
	   else if(xstart == (xmax-1) && ystart == (ymax-1))
	   {
		   U = DTarray[xstart-1][ystart]; 
		   L = DTarray[xstart][ystart-1];
		   if(U<L)
		   {
			   xstart--;
			  if(theta == -90)
			  {
				//rotate 180c and move one step
				  Buffer='d';
					//S->WriteBuffer(&Buffer,1);
			  }
				else if(theta == 180)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			  }
				else if(theta == 0)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 90;
		   }
		   else
		   {
			   ystart--;
			 if(theta == 90)
			 {
				//rotate 90a and move one step
				 Buffer='a';
				 //S->WriteBuffer(&Buffer,1);
			 }
				else if(theta == -90)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			 }
				else if(theta == 0)
				{
				//rotate 180c and move one step
					Buffer='d';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 180;
		   }
		   return Buffer;
	   }
	   else
	   {
		   if(xstart == 0)
		   {
			   D = DTarray[xstart+1][ystart];
			   R = DTarray[xstart][ystart+1];
			   L = DTarray[xstart][ystart-1];
			   if(D<R)
			   {
				   if(D<L)
				   {
					   xstart++;
					  if(theta == 90)
					  {
				//rotate 180c and move one 
						  Buffer='d';
						 // S->WriteBuffer(&Buffer,1);
					  }
					  else if(theta == 180)
					  {
				//rotate 90a and move one step
						  Buffer='a';
						  //S->WriteBuffer(&Buffer,1);
					  }
					  else if(theta == 0)
					  {
				//rotate 90c and move one step
						  Buffer='c';
						  //S->WriteBuffer(&Buffer,1);
					  }
						theta = -90;
				   }
				   else
				   {
					   ystart--;
					   if(theta == 90)
					   {
				//rotate 90a and move one step
						   Buffer='a';
						  // S->WriteBuffer(&Buffer,1);
					   }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
							//S->WriteBuffer(&Buffer,1);
					   }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
							//S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
				   }
			   }
			   else
			   {
				   if(R<L)
				   {
					   ystart++;
					 if(theta == 90)
					 {
				//rotate 90c and move one step
						 Buffer='c';
						//S->WriteBuffer(&Buffer,1);
					 }
						else if(theta == 180)
						{
							//rotate 180c and move one step
							Buffer='d';
							//S->WriteBuffer(&Buffer,1);
					 }
					else if(theta == -90)
					{
				//rotate 90a and move one step
						Buffer='a';
						//S->WriteBuffer(&Buffer,1);
						}
					theta = 0;
				   }
				   else
				   {
					   ystart--;
					  if(theta == 90)
					  {
				//rotate 90a and move one step
						  Buffer='a';
						//S->WriteBuffer(&Buffer,1);
					  }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
					  }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
							//S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
				   }
			   }
			   return Buffer;
		   }
		   else if(xstart == (xmax-1))
		   {
			   U = DTarray[xstart-1][ystart];
			   R = DTarray[xstart][ystart+1];
			   L = DTarray[xstart][ystart-1];
			   if(U<R)
			   {
				   if(U<L)
				   {
					   xstart--;
			   if(theta == -90)
			   {
				//rotate 180c and move one step
				   Buffer='d';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 180)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 0)
				{
				//rotate 90a and move one step
					Buffer='a';
				//	S->WriteBuffer(&Buffer,1);
				}
				theta = 90;
				   }
				   else
				   {
					   ystart--;
					   if(theta == 90)
					   {
				//rotate 90a and move one step
						   Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
					   }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
					   }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
				   }
			   }
			   else
			   {
				   if(R<L)
				   {
					   ystart++;
					   if(theta == 90)
					   {
				//rotate 90c and move one step
						   Buffer='c';
							//S->WriteBuffer(&Buffer,1);
					   }
						else if(theta == 180)
						{
				//rotate 180c and move one step
							Buffer='d';
							//S->WriteBuffer(&Buffer,1);
					   }
					else if(theta == -90)
					{
				//rotate 90a and move one step
						Buffer='a';
							//S->WriteBuffer(&Buffer,1);
						}
					theta = 0;
				   }
				   else
				   {
					   ystart--;
					      if(theta == 90)
						  {
				//rotate 90a and move one step
							  Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
						  }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
						  }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
				   }
			   }
			   return Buffer;
		   }
		   else if(ystart == 0)
		   {
			   U = DTarray[xstart-1][ystart];
			   D = DTarray[xstart+1][ystart];
			   R = DTarray[xstart][ystart+1];
			   if(U<D)
			   {
				   if(U<R)
				   {
					   xstart--;
			   if(theta == -90)
			   {
				//rotate 180c and move one step
				   Buffer='d';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 180)
				{
				//rotate 90c and move one step
					Buffer='c';
				//	S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 0)
				{
				//rotate 90a and move one step
					Buffer='a';
				//	S->WriteBuffer(&Buffer,1);
				}
				theta = 90;
				   }
				   else
				   {
					   ystart++;
					  if(theta == 90)
					  {
				//rotate 90c and move one step
						  Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
					  }
						else if(theta == 180)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
					  }
					else if(theta == -90)
					{
				//rotate 90a and move one step
						Buffer='a';
						//S->WriteBuffer(&Buffer,1);
						}
					theta = 0;
				   }
			   }
			   else
			   {
				   if(D<R)
				   {
					   xstart++;
					    if(theta == 90)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
					  else if(theta == 180)
					  {
				//rotate 90a and move one step
						  Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
						}
					  else if(theta == 0)
					  {
				//rotate 90c and move one step
						  Buffer='c';
							//S->WriteBuffer(&Buffer,1);
					  }
						theta = -90;
				   }
				   else
				   {
					   ystart++;
					    if(theta == 90)
						{
				//rotate 90c and move one step
							Buffer='c';
							//S->WriteBuffer(&Buffer,1);
						}
						else if(theta == 180)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
					else if(theta == -90)
					{
				//rotate 90a and move one step
						Buffer='a';
					//	S->WriteBuffer(&Buffer,1);
						}
					theta = 0;
				   }
			   }
			   return Buffer;
		   }
		   else if(ystart == (ymax-1))
		   {
			   U = DTarray[xstart-1][ystart];
			   D = DTarray[xstart+1][ystart];
			   L = DTarray[xstart][ystart-1];
			   if(U<D)
			   {
				   if(U<L)
				   {
					   xstart--;
			 if(theta == -90)
			 {
				//rotate 180c and move one step
				 Buffer='d';
				//S->WriteBuffer(&Buffer,1);
			 }
				else if(theta == 180)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			 }
				else if(theta == 0)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 90;
				   }
				   else
				   {
					   ystart--;
					     if(theta == 90)
						 {
				//rotate 90a and move one step
							 Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
						 }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
						 }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
				   }
			   }
			   else
			   {
				   if(D<L)
				   {
					   xstart++;
					    if(theta == 90)
						{
				//rotate 180c and move one step
							Buffer='d';
							//S->WriteBuffer(&Buffer,1);
						}
					  else if(theta == 180)
					  {
				//rotate 90a and move one step
						  Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
						}
					  else if(theta == 0)
					  {
				//rotate 90c and move one step
						  Buffer='c';
						//S->WriteBuffer(&Buffer,1);
					  }
						theta = -90;
				   }
				   else
				   {
					   ystart--;
					     if(theta == 90)
						 {
				//rotate 90a and move one step
							 Buffer='a';
							//S->WriteBuffer(&Buffer,1);
						 }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
						 }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
				   }
			   }
			   return Buffer;
		   }
		   else
		   {
			   U = DTarray[xstart-1][ystart];
			   D = DTarray[xstart+1][ystart];
			   R = DTarray[xstart][ystart+1];
			   L = DTarray[xstart][ystart-1];
			   if(U<D)
			   {
				   if(U<R)
				   {
					   if(U<L)
					   {
						   xstart--;
			   if(theta == -90)
			   {
				//rotate 180c and move one step
				   Buffer='d';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 180)
				{
				//rotate 90c and move one step
					Buffer='c';
					//S->WriteBuffer(&Buffer,1);
			   }
				else if(theta == 0)
				{
				//rotate 90a and move one step
					Buffer='a';
					//S->WriteBuffer(&Buffer,1);
				}
				theta = 90;
					   }
					   else
					   {
						   ystart--;
						      if(theta == 90)
							  {
				//rotate 90a and move one step
								  Buffer='a';
							    //  S->WriteBuffer(&Buffer,1);
							  }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
							  }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
					   }
				   }
				   else
				   {
					   if(R<L)
					   {
						   ystart++;
						    if(theta == 90)
							{
				//rotate 90c and move one step
								Buffer='c';
							//S->WriteBuffer(&Buffer,1);
							}
						else if(theta == 180)
						{
				//rotate 180c and move one step
							Buffer='d';
					//		S->WriteBuffer(&Buffer,1);
							}
					else if(theta == -90)
					{
				//rotate 90a and move one step
						Buffer='a';
						//S->WriteBuffer(&Buffer,1);
						}
					theta = 0;
					   }
					   else
					   {
						   ystart--;
						      if(theta == 90)
							  {
				//rotate 90a and move one step
								  Buffer='a';
								//  S->WriteBuffer(&Buffer,1);
							  }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
							//S->WriteBuffer(&Buffer,1);
							  }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
					   }
				   }
			   }
			   else
			   {
				   if(D<R)
				   {
					   if(D<L)
					   {
						   xstart++;
						   if(theta == 90)
						   {
				//rotate 180c and move one step
							   Buffer='c';
							//S->WriteBuffer(&Buffer,1);
						   }
					  else if(theta == 180)
					  {
				//rotate 90a and move one step
						  Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
						   }
					  else if(theta == 0)
					  {
				//rotate 90c and move one step
						  Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
					  }
						theta = -90;
					   }
					   else
					   {
						   ystart--;
						     if(theta == 90)
							 {
				//rotate 90a and move one step
								 Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
							 }
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
						//	S->WriteBuffer(&Buffer,1);
							 }
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
							//S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
					   }
				   }
				   else
				   {
					   if(R<L)
					   {
						   ystart++;
						     if(theta == 90)
							 {
				//rotate 90c and move one step
								 Buffer='c';
							 //  S->WriteBuffer(&Buffer,1);
							 }
						else if(theta == 180)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
							 }
					else if(theta == -90)
					{
				//rotate 90a and move one step
						Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
						}
					theta = 0;
					   }
					   else
					   {
						   ystart--;
						    if(theta == 90)
							{
				//rotate 90a and move one step
								Buffer='a';
						//	S->WriteBuffer(&Buffer,1);
							}
						else if(theta == -90)
						{
				//rotate 90c and move one step
							Buffer='c';
							//S->WriteBuffer(&Buffer,1);
							}
						else if(theta == 0)
						{
				//rotate 180c and move one step
							Buffer='d';
						//	S->WriteBuffer(&Buffer,1);
						}
						theta = 180;
					   }
				   }
			   }
			   return Buffer;
		   }
	   } 
	 }
	 return 's';
}

void PathDirector::UpdateDirection(SharedResource*Resource)
{
	State=ReadyForPathPlanning;
	while(true)
	{
		if(Resource->GetSignalsToExit(6)==true)
		{
			Resource->SetSignalsToExit(false,6);
			break;
		}
		else if(Resource->GetManualControl()==false && Resource->GetGoingToDestination()==true && State==ReadyForPathPlanning)
		{
			GridMap=Resource->GetMap();
			if(GridMap->getArea()==0)//un-initialized Map
			{
				mrpt::system::sleep(10);
				continue;
			}
			Resource->GetAbsolutePosition(AbsolutePosition[0],AbsolutePosition[1],AbsolutePosition[2]);
			float DestinationLocationX;//in meters
			float DestinationLocationY;//in meters
			Resource->GetDestinationLocation(DestinationLocationX,DestinationLocationY);

			int xstart=GridMap->x2idx(AbsolutePosition[0]);//meters to map coordinates
			int ystart=GridMap->y2idx(AbsolutePosition[1]);
			int xgoal=GridMap->x2idx(DestinationLocationX);
			int ygoal=GridMap->y2idx(DestinationLocationY);

			InitializeInterfaces();
			
			//Some Code To Plan Path and Update Direction Variable 
			initialize(xstart,ystart);
			for(int i = 0;i<xmax;i++)
				{
					forwardscan(map,xstart,ystart);
					reversescan(map,xstart,ystart);
				}

			Direction=DTPP(xgoal,ygoal,xstart,ystart);

			//Delete Map and DtArray after usage
			for (int i=0;i<xmax;i++)
			{
				delete [] map[i];
				delete [] DTarray[i];
			}
			delete [] map;
			delete [] DTarray;
			map=NULL;
			DTarray=NULL;

			//begin section for motion Control
			for(int i=0;i<3;i++)
				RefrenceForStepping[i]=AbsolutePosition[i];

			switch (Direction)
			{
			case 'a':
				State=ExecutingAntiClockwise90Deg;
				break;
			case 'c':
				State=ExecutingClockwise90Deg;
				break;
			case 'd':
				State=ExecutingAntiClockwise180Deg;
				Direction='a';
				break;
			case 'f':
				State=ExecutingForward;
				break;
			case 's':
				State=ReadyForPathPlanning;
				std::cout<<std::endl<<"Destination reached"<<std::endl<<"Your are now in Manual Control Mode"<<std::endl;
				Resource->SetManualControl(true);
				Resource->SetGoingToDestination(false);
				break;
			}

			Resource->SetDirection(Direction);//sending direction to shared resource
		}
		else if(Resource->GetManualControl()==false && Resource->GetGoingToDestination()==true && State!=ReadyForPathPlanning)//in the motion execution senario
		{
			Resource->GetAbsolutePosition(AbsolutePosition[0],AbsolutePosition[1],AbsolutePosition[2]);
			double IncrementalMotion[3];
			for(int i=0;i<3;i++)
				IncrementalMotion[i]=AbsolutePosition[i]-RefrenceForStepping[i];
			/*
			if(State==ExecutingAntiClockwise180Deg || State==ExecutingAntiClockwise180Deg)
			{
				IncrementalMotion[2]=AbsolutePosition[2]-RefrenceForStepping[2];
			}
			else if(State==ExecutingClockwise90Deg)
			{
				IncrementalMotion[2]=RefrenceForStepping[2]-AbsolutePosition[2];
			}
			
			if(IncrementalMotion[2] <0)//make angle fro 0 to 2*pi
				IncrementalMotion[2]=IncrementalMotion[2]+M_2PI;
			*/
			//continue this switch later
			switch(State)
			{
			case ExecutingAntiClockwise90Deg://use Pi/2 constant defined in mrpt
				if(IncrementalMotion[2]>M_PI/2 || IncrementalMotion[2]<-3*M_PI/2  )
				{
					State=ReadyForPathPlanning;
					Direction='f';
					Resource->SetDirection(Direction);
				}
				break;
			case ExecutingClockwise90Deg:
				if(IncrementalMotion[2]<-M_PI/2 || (IncrementalMotion[2]< 3*M_PI/2 && IncrementalMotion[2] > 0) )
				{
					State=ReadyForPathPlanning;
					Direction='f';
					Resource->SetDirection(Direction);
				}
				break;
			case ExecutingAntiClockwise180Deg:
				if(IncrementalMotion[2]>M_PI || IncrementalMotion[2]<-M_PI )
				{
					State=ReadyForPathPlanning;
					Direction='f';
					Resource->SetDirection(Direction);
				}
				break;
			case ExecutingForward:
				if( (pow(IncrementalMotion[0],2)+pow(IncrementalMotion[1],2)) > 0.07*0.07)//unCertain 
				{
					State=ReadyForPathPlanning;
					Direction='f';
					Resource->SetDirection(Direction);
				}
				break;
			}
			
		}
		else//something strange happened or the user exited the mode in the middle of it's operation  or the mode is not Going to destination
		{
			State=ReadyForPathPlanning;
			mrpt::system::sleep(30);
		}
	}
}