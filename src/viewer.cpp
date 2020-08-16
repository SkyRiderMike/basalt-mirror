#include <basalt/vio_viewer/vio_viewer.h>

int main(/* int argc, char** argv */)
{
    RobotA::VioViewer viewer;
    
    viewer.Start();
    viewer.Wait();
}