/**
	localizer.cpp

	Purpose: implements a 2-dimensional histogram filter
	for a robot living on a colored cyclical grid by
	correctly implementing the "initialize_beliefs",
	"sense", and "move" functions.

	This file is incomplete! Your job is to make these
	functions work. Feel free to look at localizer.py
	for working implementations which are written in python.
*/

#include "helpers.cpp"
#include <stdlib.h>
#include "debugging_helpers.cpp"

using namespace std;

/**
    Initializes a grid of beliefs to a uniform distribution.

    @param grid - a two dimensional grid map (vector of vectors
    	   of chars) representing the robot's world. For example:

    	   g g g
    	   g r g
    	   g g g

		   would be a 3x3 world where every cell is green except
		   for the center, which is red.

    @return - a normalized two dimensional grid of floats. For
           a 2x2 grid, for example, this would be:

           0.25 0.25
           0.25 0.25
*/
vector< vector <float> > initialize_beliefs(vector< vector <char> > grid) {

	// Initialize size and area.
	int height = grid.size();
	int width = grid[0].size();
	int area = height * width;

	// Initialize belief per cell.
	float belief_per_cell = 1.0 / area;

	// Built initial beliefs grid -- with a uniform probability distribution.
	vector < vector <float> > newGrid;

	for (int row = 0; row < height; row++) {
		vector<float> newRow;
		for (int col = 0; col < width; col++) {
			newRow.push_back(belief_per_cell);
		}
		newGrid.push_back(newRow);
	}

	return newGrid;
}

/**
    Implements robot sensing by updating beliefs based on the
    color of a sensor measurement

		@param color - the color the robot has sensed at its location

		@param grid - the current map of the world, stored as a grid
			   (vector of vectors of chars) where each char represents a
			   color. For example:

			  	g g g
	    	  g r g
	    	  g g g

   	@param beliefs - a two dimensional grid of floats representing
   		   the robot's beliefs for each cell before sensing. For
   		   example, a robot which has almost certainly localized
   		   itself in a 2D world might have the following beliefs:

   		   0.01 0.98
   		   0.00 0.01

    @param p_hit - the RELATIVE probability that any "sense" is
    	   correct. The ratio of p_hit / p_miss indicates how many
    	   times MORE likely it is to have a correct "sense" than
    	   an incorrect one.

   	@param p_miss - the RELATIVE probability that any "sense" is
    	   incorrect. The ratio of p_hit / p_miss indicates how many
    	   times MORE likely it is to have a correct "sense" than
    	   an incorrect one.

    @return - a normalized two dimensional grid of floats
    	   representing the updated beliefs for the robot.
*/
vector< vector <float> > sense(char color,
	vector< vector <char> > grid,
	vector< vector <float> > beliefs,
	float p_hit,
	float p_miss)
{
	// Build new beliefs grid by multiplying each cell by the hit or miss coefficient.
	vector< vector <float> > newBeliefs;

	for (int row = 0; row < grid.size(); row++) {
		vector<float> newRow;
		for (int col = 0; col < grid[0].size(); col++) {
			// If sensed color and world color match...
			if (color == grid[row][col]) {
				// ... it's a hit -- multiply accordingly.
				newRow.push_back(beliefs[row][col] * p_hit);
			}
			else {
				// ... it's a miss -- multiply accordingly.
				newRow.push_back(beliefs[row][col] * p_miss);
			}
		}
		newBeliefs.push_back(newRow);
	}

	return normalize(newBeliefs);
}

/**
		Implements robot motion by updating beliefs based on the
    intended dx and dy of the robot.

    For example, if a localized robot with the following beliefs

    0.00  0.00  0.00
    0.00  1.00  0.00
    0.00  0.00  0.00

    and dx and dy are both 1 and blurring is 0 (noiseless motion),
    than after calling this function the returned beliefs would be

    0.00  0.00  0.00
    0.00  0.00  0.00
    0.00  0.00  1.00

	@param dy - the intended change in y position of the robot

	@param dx - the intended change in x position of the robot

 	@param beliefs - a two dimensional grid of floats representing
 		   the robot's beliefs for each cell before sensing. For
 		   example, a robot which has almost certainly localized
 		   itself in a 2D world might have the following beliefs:

 		   0.01 0.98
 		   0.00 0.01

  @param blurring - A number representing how noisy robot motion
         is. If blurring = 0.0 then motion is noiseless.

  @return - a normalized two dimensional grid of floats
  	   representing the updated beliefs for the robot.
*/
vector< vector <float> > move(int dy, int dx,
	vector < vector <float> > beliefs,
	float blurring)
{
	// Initialize beliefs grid size.
	int height = beliefs.size();
	int width = beliefs[0].size();

	// Create new grid of same size, and fill it with zeros.
	vector < vector <float> > newGrid;

	for (int row = 0; row < height; row++) {
		vector<float> newRow;
		for (int col = 0; col < width; col++) {
			newRow.push_back(0.0);
		}
		newGrid.push_back(newRow);
	}

	// Get beliefs grid values one by one, to place them on new grid.
	// Shift them all to a new position based on the robot movement.
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			// Since C++ vectors don't accept negative indices, here's how we can
			// deal with modulus of negative indices: http://bit.ly/2LlO77i
			int new_row = ((row + dy) % height + height) % height;
			int new_col = ((col + dx) % width + width) % width;
			// Move beliefs to their new position on the new grid.
			newGrid[new_row][new_col] = beliefs[row][col];
		}
	}

	return blur(newGrid, blurring);
}
