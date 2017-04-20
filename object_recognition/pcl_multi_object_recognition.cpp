#include <thread>

#include "find_function.h"
#include "OpenniStreamer.h"
#include "Visualizer.h"



int
main (int argc, char** argv)
{


  // Parse input and set algorithm variables
  ParseCommandLine (argc, argv);
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr complete_scene (new pcl::PointCloud<PointType> ());

  std::vector<float> * filters = new std::vector<float>();
  std::vector<int> * icp_iterations = new std::vector<int>();
  // Load the input model (n models but for now only one is used)
  std::vector < pcl::PointCloud < PointType > ::Ptr > model_list = ReadModels (argv, *filters, *icp_iterations);
  if (model_list.size () == 0) {
    std::cout << " no models loaded " << std::endl;
    return 1;
  }

  // Check if an oni file is specified as input or if the camera stream has to be used and initialize the correct stream.
  OpenniStreamer * openni_streamer;
  if(!oni_file.empty())
    openni_streamer = new OpenniStreamer(oni_file);
  else
    openni_streamer = new OpenniStreamer();
  // Initialize all the fundamental datastructures
  int num_threads = model_list.size();
  Visualizer visualizer;
  Semaphore s(num_threads);
  std::vector<std::thread> thread_list(num_threads);
  std::vector<ClusterType> found_models(num_threads);
  ErrorWriter e;
  int frame_index;

  // Read first frame and launch the threads
  if(!openni_streamer->HasDataLeft())
      exit(0);
  scene = openni_streamer->GetCloud();
  copyPointCloud (*scene, *complete_scene);
  frame_index = openni_streamer->GetFrameIndex();

  for(int i = 0; i < num_threads; ++i)
      thread_list[i] = std::thread(FindObject, model_list[i], std::ref(scene), std::ref(s), std::ref(found_models), i, filters->at(i), icp_iterations->at(i), std::ref(e), std::ref(frame_index));
    
  // Start the main detection loop
  // 1- wait for the threads to find all the objects
  // 2- visualize the scene and the found models
  // 3- read a new scene
  // 4- wake up threads
    
  while(!visualizer.viewer_.wasStopped ()){
    //wait for the threads to complete
    s.Wait4threads();
    
    //Visualizing the model, scene and the estimated model position
    SetViewPoint (complete_scene);
    visualizer.Visualize (model_list, found_models, complete_scene);
    
    // Grab a frame and create the pointcloud checking in case if the oni stream is finished
    if(!openni_streamer->HasDataLeft())
      break;
    scene = openni_streamer->GetCloud();
    frame_index = openni_streamer->GetFrameIndex();
    copyPointCloud (*scene, *complete_scene);
    // Wake up the threads
    s.Notify2threads();
  }

  // Notifies all the threads top stop and waits for them to join
  s.SetStop();
  for(int i = 0; i < num_threads; ++i)
      thread_list[i].join();

  return (0);
}

