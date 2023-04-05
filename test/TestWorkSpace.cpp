#include <cassert>
#include <iostream>

#include "../src/WorkSpace.hpp"

int main(){
    // Make obstacles
    //Obstacle 1 
    Obstacle o1 = Obstacle(3,3,1,1);
    
    WorkSpace W = WorkSpace(0,10,0,10);
    W.addObstacle(o1);

    // Check_Collision
    assert(W.Check_collision(3, 3.5));
    assert(W.Check_collision(3.5, 3.5));
    assert(W.Check_collision(2, 2));
    assert(W.Check_collision(2, 4));
    assert(W.Check_collision(4, 4));
    assert(W.Check_collision(4, 2));

    // containObstacle(Region r)
    Region r_test1 = Region(1,1,3,3); 
    assert(W.containObstacle(r_test1));
    Region r_test2 = Region(2.5,2.5,0.5,0.5); 
    assert(W.containObstacle(r_test2));
    Region r_test3 = Region(4, 4,0.5,0.5); 
    assert(W.containObstacle(r_test3));
    Region r_test4 = Region(4.1, 4.1,0.5,0.5); 
    assert(!W.containObstacle(r_test4));

    // SplitRegion(Region r)
    Region sr_test1 = Region(4,4,8,8);
    std::vector<Region> resutl1 = W.SplitRegion(sr_test1);
    assert(resutl1.size() == 4);
    assert(resutl1[0].x_start == 4);
    assert(resutl1[0].x_extent == 4);
    

    WorkSpace W2 = WorkSpace(0,0,10,10);
    W2.Decompose();
    assert(W2.countRegionSize() == 1);


}