#include <swarm-suv-control/controller/follow.hpp>
//#include <swarm-suv-control/collisionavoidance.hpp>

//NOTE: adding
#include <swarm-suv-control/collisionavoidance.hpp>

namespace swarm {
namespace controller {

Follow::Follow()
    : _nh("follow"),
      _stationId(0),
      //_pCa(new CollisionAvoidance),
      _rate(100)
{
  DCHECK();
  ros::NodeHandle nh("~");
  nh.param("station", _stationId, 0);

  DCHECK();
  if (0 >= _stationId || 6 < _stationId)
    throw std::runtime_error("error");

  DCHECK();
  _vec.push_back(std::make_shared<swarm::drone::Leader>(_stationId));


  DCHECK();
  for(int i=1; i<=5; ++i)
    _vec.push_back(std::make_shared<swarm::drone::Follower>(_stationId, i));

}

Follow::~Follow()
{
  //!NOTE: Adding
  //delete _pCa;
  //_pCa = nullptr;
  DCHECK();
}

void
Follow::Spin()
{
  DCHECK();

  while(ros::ok())
  {
     int sys_id =0;

    for(auto it=_vec.begin(); it!=_vec.end(); ++it)
    {

      double StateXarray[5]; //you have to correctly insert the array number([5])
      double StateYarray[5];

      int number = 1; 
  
         for(auto iter=_vec.begin(); iter!=_vec.end(); ++iter)
         {
   
           if(iter == _vec.begin())
           {
              StateXarray[0]  = (*iter) -> LeaderGetStateX();
              StateYarray[0]  = (*iter) -> LeaderGetStateY();
              int number = 1; 
           }

           else if(iter != _vec.begin())
           {
              StateXarray[number] = (*iter) ->FollowerGetStateX();
              StateYarray[number] = (*iter) ->FollowerGetStateY();
           
             ++number;
           }
           
         }

       
      //NOTE::Calculate the relative distance 
       double RelativeDistance[5];

       GetRelativeDistance(StateXarray, StateYarray, RelativeDistance, sys_id);  
         //std::cout<<"relative_distance[0]"<<RelativeDistance[0]<<std::endl;
         //std::cout<<"relative_distance[1]"<<RelativeDistance[1]<<std::endl;

      //NOTE::Obtain the minimum value from the relative distance
       double Min = MinValue_fuction(RelativeDistance);
      
       std::cout<< "Min" << Min<<std::endl;

       //CollisionAvoidance::GetMinimumValue(Min);
       //CallCollisionAvoidance(Min);

#if 1
      CollisionAvoidance::SetMinimumValue(Min);
#else
      std::cout<<CollisionAvoidance::GetMinimumValue()<<std::endl;
#endif
       ++sys_id;
     

    
       /*

       std::cout<< "StateXarrayX[0]" << StateXarray[0]<<std::endl;
       std::cout<< "StateXarrayY[0]" << StateYarray[0]<<std::endl;
       std::cout<< "StateXarrayX[1]" << StateXarray[1]<<std::endl;
       std::cout<< "StateXarraY[1]" << StateYarray[1]<<std::endl;
       std::cout<< "StateXarrayX[2]" << StateXarray[2]<<std::endl;
       std::cout<< "StateXarraY[2]" << StateYarray[2]<<std::endl;
       std::cout<< "StateXarrayX[3]" << StateXarray[3]<<std::endl;
       std::cout<< "StateXarraY[3]" << StateYarray[3]<<std::endl;
       std::cout<< "StateXarrayX[4]" << StateXarray[4]<<std::endl;
       std::cout<< "StateXarraY[4]" << StateYarray[4]<<std::endl;
       std::cout<< "StateXarrayX[5]" << StateXarray[5]<<std::endl;
       std::cout<< "StateXarraY[5]" << StateYarray[5]<<std::endl;
*/



       

      (*it)->Spin();
    }

    ros::spinOnce();
    _rate.sleep();
    
  }
#if 0
  ros::waitForShutdown();

  ROS_INFO("Stopping test");
#endif
}



double 
Follow::GetRelativeDistance(double *StateXarray, 
                           double *StateYarray, 
                           double *RelativeDistance, int sys_id)
                       
{

    switch(sys_id)
    {
        case 0:
            RelativeDistance[0] = -1;
            RelativeDistance[1] = 10;
            RelativeDistance[2] = 2;
            RelativeDistance[3] = 3;
            RelativeDistance[4] = 4;
        break;


   case 1:

   RelativeDistance[0] = sqrt(pow(StateXarray[1] -StateXarray[0],2) + pow(StateYarray[1] -StateYarray[0],2));
   RelativeDistance[1] = sqrt(pow(StateXarray[1] -StateXarray[2],2) + pow(StateYarray[1] -StateYarray[2],2));
   RelativeDistance[2] = sqrt(pow(StateXarray[1] -StateXarray[3],2) + pow(StateYarray[1] -StateYarray[3],2)); 
   RelativeDistance[3] = sqrt(pow(StateXarray[1] -StateXarray[4],2) + pow(StateYarray[1] -StateYarray[4],2));
   RelativeDistance[4] = sqrt(pow(StateXarray[1] -StateXarray[5],2) + pow(StateYarray[1] -StateYarray[5],2));


    /*
   std::cout<<"relative_distance[0]"<<RelativeDistance[0]<<std::endl;

    
            RelativeDistance[0] = -2;
            RelativeDistance[1] = 10;
            RelativeDistance[2] = 100;
            RelativeDistance[3] = 200;
            RelativeDistance[4] = 4;

            */
  break;

 

  case 2:



   RelativeDistance[0] = sqrt(pow(StateXarray[2] -StateXarray[0],2) + pow(StateYarray[2] -StateYarray[0],2));
   RelativeDistance[1] = sqrt(pow(StateXarray[2] -StateXarray[1],2) + pow(StateYarray[2] -StateYarray[1],2));
   RelativeDistance[2] = sqrt(pow(StateXarray[2] -StateXarray[3],2) + pow(StateYarray[2] -StateYarray[3],2)); 
   RelativeDistance[3] = sqrt(pow(StateXarray[2] -StateXarray[4],2) + pow(StateYarray[2] -StateYarray[4],2));
   RelativeDistance[4] = sqrt(pow(StateXarray[2] -StateXarray[5],2) + pow(StateYarray[2] -StateYarray[5],2));

     /*
            RelativeDistance[0] = 3;
            RelativeDistance[1] = -3;
            RelativeDistance[2] = 100;
            RelativeDistance[3] = 200;
            RelativeDistance[4] = 4;

            */
  break;

 

  case 3:

   RelativeDistance[0] = sqrt(pow(StateXarray[3] -StateXarray[0],2) + pow(StateYarray[3] -StateYarray[0],2));
   RelativeDistance[1] = sqrt(pow(StateXarray[3] -StateXarray[1],2) + pow(StateYarray[3] -StateYarray[1],2));
   RelativeDistance[2] = sqrt(pow(StateXarray[3] -StateXarray[2],2) + pow(StateYarray[3] -StateYarray[2],2)); 
   RelativeDistance[3] = sqrt(pow(StateXarray[3] -StateXarray[4],2) + pow(StateYarray[3] -StateYarray[4],2));
   RelativeDistance[4] = sqrt(pow(StateXarray[3] -StateXarray[5],2) + pow(StateYarray[3] -StateYarray[5],2));
   

   /*
            RelativeDistance[0] = 4;
            RelativeDistance[1] = 4;
            RelativeDistance[2] = 100;
            RelativeDistance[3] = 200;
            RelativeDistance[4] = 5;
            */
  break;

  

  case 4:


   RelativeDistance[0] = sqrt(pow(StateXarray[4] -StateXarray[0],2) + pow(StateYarray[4] -StateYarray[0],2));
   RelativeDistance[1] = sqrt(pow(StateXarray[4] -StateXarray[1],2) + pow(StateYarray[4] -StateYarray[1],2));
   RelativeDistance[2] = sqrt(pow(StateXarray[4] -StateXarray[2],2) + pow(StateYarray[4] -StateYarray[2],2)); 
   RelativeDistance[3] = sqrt(pow(StateXarray[4] -StateXarray[3],2) + pow(StateYarray[4] -StateYarray[3],2));
   RelativeDistance[4] = sqrt(pow(StateXarray[4] -StateXarray[5],2) + pow(StateYarray[4] -StateYarray[5],2));


   /*
             RelativeDistance[0] = 5;
            RelativeDistance[1] = 5;
            RelativeDistance[2] = 100;
            RelativeDistance[3] = 200;
            RelativeDistance[4] = 6;  */
  break;

 

  case 5:
  
   RelativeDistance[0] = sqrt(pow(StateXarray[5] -StateXarray[0],2) + pow(StateYarray[5] -StateYarray[0],2));
   RelativeDistance[1] = sqrt(pow(StateXarray[5] -StateXarray[1],2) + pow(StateYarray[5] -StateYarray[1],2));
   RelativeDistance[2] = sqrt(pow(StateXarray[5] -StateXarray[2],2) + pow(StateYarray[5] -StateYarray[2],2)); 
   RelativeDistance[3] = sqrt(pow(StateXarray[5] -StateXarray[3],2) + pow(StateYarray[5] -StateYarray[3],2));
   RelativeDistance[4] = sqrt(pow(StateXarray[5] -StateXarray[4],2) + pow(StateYarray[5] -StateYarray[4],2));



   /*
            RelativeDistance[0] = 6;
            RelativeDistance[1] = 7;
            RelativeDistance[2] = 100;
            RelativeDistance[3] = 200;
            RelativeDistance[4] = 9;

            */
  break;
   

  
     

    }

} 




double
Follow::MinValue_fuction(double * RelativeDistance)
{

//!NOTE: TODO
  double Min = RelativeDistance[0];

   for (int i=0; i<5; ++i) // you shoud define i data type such as int, double also auto (you must 0<4 not 0<5 because the form is x[0],x[1],x[2],x[3])
   {
       if(Min > RelativeDistance[i])
       {
         Min = RelativeDistance[i];
       }   
   }
           return Min;

}



} // namespace swarm::Drone
} // namespace swarm


double 
swarm::controller::Follow::CallCollisionAvoidance(double min)
{

 //CollisionAvoidance::GetMinimumValue(min);

return min;

}



