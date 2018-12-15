#include "./Explorer.hpp"

#ifndef TERRAPINAVIGATOR_INCLUDE_TERRAPINAVIGATOR_TURTLE_H_
#define TERRAPINAVIGATOR_INCLUDE_TERRAPINAVIGATOR_TURTLE_H_
/**
 * @brief Agv class
 *
 * Initializes subscriber, publishers and timers
 * Has a method to publish twist messges.
 *
 */
class Agv {
 public:
    /**
     * @brief Constructor for Agv Class
     */
    Agv();
    /**
     *
     * @brief Publishes twist messages
     *
     * @return true if successful
     *
     */
    bool explore();
    /**
     * @brief Destructor for Agv Class
     */
    ~Agv();

 private:
    ros::NodeHandle n;
    /**
      * @brief Creates Explorer Object
      */
    Explorer Explorer = Explorer();
    /**
     * @brief registers subscriber for image pointers
     */
    ros::Subscriber subLaserScan;
    /**
     * @brief registers timer for rotating the turtleBot
     */
    ros::Timer Rotatetimer;
    /**
     * @brief registers timer for camera service
     */
    ros::Timer camTimer;
    /**
     * @brief registers publisher for action(twist) messages
     */
    ros::Publisher actionPub;
};

#endif /* TERRAPINAVIGATOR_INCLUDE_TERRAPINAVIGATOR_TURTLE_H_ */
