// Includes a core set of basic Gazebo functions
#include <gazebo/gazebo.hh>
 
// All plugins must be in the gazebo namespace
namespace gazebo
{
  // Each plugin has to inherit from a plugin type.
  // In this case, we are inheriting from the WorldPlugin type.
  class WorldPluginTutorial : public WorldPlugin
  {
    public: WorldPluginTutorial() : WorldPlugin()
            {
              printf("Hello World!\n");
            }
 
    // This function is mandatory. It receives an element in 
    // Simulation Description Format (SDF) that contains the elements
    // and attributes that are specified in the loaded SDF file.
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  // Register the plugin with the simulator.
  // Note that there are matching register macros for the six types
  // of plugins in Gazebo.
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}