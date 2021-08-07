
#ifndef RESET_UR3_H
#define RESET_UR3_H


#include <iostream>
#include <vector>
#include <array>

#include <ros/ros.h>


namespace ur3{

    class ResetUR3 {
        public:
            ResetUR3(ros::NodeHandle& node);
            ~ResetUR3() {};

            void desconnectUR3();
            
        private:
            

    };

}

#endif