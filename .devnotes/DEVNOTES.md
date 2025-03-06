## NODE
- how to implement diagnostic updater that is already existing in ros2 ?
- Validate node functionality with native ROS2 nodes

// TODO: Do we need stack and attributes? zephyr examples does not use them
// pthread_attr_t thread_attr {};
// size_t thread_stacksz = K_THREAD_STACK_SIZEOF(thread_area);
// int rc = pthread_attr_init(&thread_attr);
// if (rc != 0 ) {
//     printk("init_analyzer_handler: pthread_attr_init failed: %d\n", errno);
//     return -1;
// }

// rc = pthread_attr_setstack(&thread_attr, &thread_area,
//                             thread_stacksz);
// if (rc != 0 ) {
//     printk("init_analyzer_handler: pthread_attr_setstack failed: %d\n", errno);
//     return -1;
// }

// int result = pthread_create(&thread_, &thread_attr, thread_entry, this);
// if (result != 0) {
//     printk("Failed to create thread for node %s\n", node_name_.c_str());
//     return -1;
// }

## SEQUENCES
- find a way to access buffers with a sequenced class including eg. trajectory.points 
- Remove the duality, use zephyr alloc for dds allocations and get rid of k_mallocs inside sequence instead just use dds_alloc
which means we all use zephyr alloc for dds allocations but on the code we will use dds_alloc and dds_free, this we will get
best of both worlds, we can use zephyr alloc for dds allocations and leave memory management to cyclonedds https://cyclonedds.io/docs/cyclonedds/latest/config/allocation-config.html
- try using idlc cpp compiler see if it is compatible with zephyr especially if introduces any overhead on memory usage

## ROS UTILS
- universal wall timer with zephyr kernel timers vs chrono timers ? Ambroise was using chrono timers

## DDS
- do we need message converter ? try building .idl messages with rosidl instead of cyclonedds idlc
- Validate cycloneDDS <-> ROS2 communication with the native ROS2 nodes
- Validate the message conversion between ROS2 and Zephyr

## MISC
- Compare launch structure and synchronization with the native ROS2 nodes
- Compare the logging and logging structure with the native ROS2 nodes

## NOTES
- try-catch should be used in critical functions, because it is hard to port the code without it
