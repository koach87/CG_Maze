/************************************************************************
     File:        Maze.cpp

     Author:     
                  Stephen Chenney, schenney@cs.wisc.edu
     Modifier
                  Yu-Chi Lai, yu-chi@cs.wisc.edu

     Comment:    
						(c) 2001-2002 Stephen Chenney, University of Wisconsin at Madison

						Class header file for Maze class. Manages the maze.
		

     Platform:    Visio Studio.Net 2003 (converted to 2005)

*************************************************************************/

#include "Maze.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <FL/Fl.h>
#include <Fl/gl.h>
#include <FL/fl_draw.h> 
#include <GL/glu.h>
#include <iostream>
#include "LineSeg.h"
#include "Edge.h"

const char Maze::X = 0;
const char Maze::Y = 1;
const char Maze::Z = 2;

const float Maze::BUFFER = 0.1f;

void ToScreen(float* mat) {
	float devide = mat[3];
	for (int i = 0; i < 4; i++) {
		mat[i] /= devide;
	}
}

//function to mult [4*4][1*4] matrix 
void multMatrix(float *a, float *b)
{
	float reg[4];
	for (int i = 0; i < 4; i++) {
		reg[i] = b[i];
	}
	for(int i =0;i<4;i++){
		b[i] = 0;
		for (int j = 0; j < 4; j++) {
			b[i] += a[i + (4 *j)] * reg[j];
		}
	}
}

void NorMatrix_(float result[3], float A[3], float B[3]) {
	result[0] = A[1] * B[2] - A[2] * B[1];
	result[1] = A[2] * B[0] - A[0] * B[2];
	result[2] = A[0] * B[1] - A[1] * B[0];
}

//Function of projection matrix
void gluPerspective__(float* mat, GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar) {

	float t, b, l, r, n, f;
	t = zNear * tanf(fovy * M_PI / 360.0);
	r = t * aspect;
	b = -t;
	l = -r;
	n = zNear;
	f = zFar;

	float matrix[16];
	for (int i = 0; i < 16; i++) {
		matrix[i] = 0.0;
	}
	matrix[0] = 2 * n / (r - l);
	matrix[5] = 2 * n / (t - b);
	matrix[8] = (r + l) / (r - l);
	matrix[9] = (t + b) / (t - b);
	matrix[10] = -(f + n) / (f - n);
	matrix[11] = -1.0;
	matrix[14] = -2 * f * n / (f - n);
	//glLoadMatrixf(matrix);
	multMatrix(matrix, mat);
}

void Normal3fv(float* f) {
	float id = sqrt((f[0] * f[0] + f[1] * f[1] + f[2] * f[2]));
	for (int i = 0; i < 3; i++) {
		f[i] /=  id;
	}
}


void gluLookAt__(float* mat, GLdouble eyex, GLdouble eyey, GLdouble eyez, GLdouble centerx, GLdouble centery, GLdouble centerz, GLdouble upx, GLdouble upy, GLdouble upz) {

	float Trans_Matrix[16] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		-eyex,-eyey,-eyez,1
	};
	multMatrix(Trans_Matrix, mat);

	float F[3] = {
	   centerx - eyex,
	   centery - eyey,
	   centerz - eyez
	};
	Normal3fv(F);

	float UP[3] = { upx,upy,upz };
	Normal3fv(UP);

	float s[3];
	NorMatrix_(s, F, UP);

	//Normal3fv(s);
	float u[3];
	//旋轉
	NorMatrix_(u, s, F);

	float matrix[16] = {
		s[0],	u[0],	-F[0],	0.0,
		s[1],	u[1],	-F[1],	0.0,
		s[2],	u[2],	-F[2],	0.0,
		0.0,	0.0,	0.0,	1.0
	};
	

	//glMultMatrixf(matrix);
	//glTranslated(-eyex, -eyey, -eyez);
	
	multMatrix(matrix, mat);
}


//**********************************************************************
//* 迷宮例外控制
// * Constructor for the maze exception
//======================================================================
MazeException::
MazeException(const char *m)
//======================================================================
{
	message = new char[strlen(m) + 4];
	strcpy(message, m);
}


//**********************************************************************
//* 預設迷宮設置
// * Constructor to create the default maze
//======================================================================
Maze::
Maze(const int nx, const int ny, const float sx, const float sy)
//======================================================================
{
	// Build the connectivity structure.
	Build_Connectivity(nx, ny, sx, sy);

	// Make edges transparent to create a maze.
	Build_Maze();

	// Set the extents of the maze
	Set_Extents();

	// Default values for the viewer.
	viewer_posn[X] = viewer_posn[Y] = viewer_posn[Z] = 0.0;
	viewer_dir = 0.0;
	viewer_fov = 45.0;

	// Always start on the 0th frame.
	frame_num = 0;
}


//**********************************************************************
//* 讀取預製好的迷宮
// * Construtor to read in precreated maze
//======================================================================
Maze::
Maze(const char *filename)
//======================================================================
{
	char    err_string[128];
	FILE    *f;
	int	    i;

	// Open the file
	if ( ! ( f = fopen(filename, "r") ) )
		throw new MazeException("Maze: Couldn't open file");

	// Get the total number of vertices
	if ( fscanf(f, "%d", &num_vertices) != 1 )
		throw new MazeException("Maze: Couldn't read number of vertices");

	// Read in each vertices
	vertices = new Vertex*[num_vertices];
	for ( i = 0 ; i < num_vertices ; i++ ) {
		float x, y;
		if ( fscanf(f, "%g %g", &x, &y) != 2 )	{
			sprintf(err_string, "Maze: Couldn't read vertex number %d", i);
			throw new MazeException(err_string);
		}
		vertices[i] = new Vertex(i, x, y);
	}

	// Get the number of edges
	if ( fscanf(f, "%d", &num_edges) != 1 )
		throw new MazeException("Maze: Couldn't read number of edges");

	// read in all edges
	edges = new Edge*[num_edges];
	for ( i = 0 ; i < num_edges ; i++ ){
		int     vs, ve, cl, cr, o;
		float	r, g, b;
		if ( fscanf(f, "%d %d %d %d %d %g %g %g",
						&vs, &ve, &cl, &cr, &o, &r, &g, &b) != 8) {
			sprintf(err_string, "Maze: Couldn't read edge number %d", i);
			throw new MazeException(err_string);
		}
		edges[i] = new Edge(i, vertices[vs], vertices[ve], r, g, b);
		edges[i]->Add_Cell((Cell*)cl, Edge::LEFT);
		edges[i]->Add_Cell((Cell*)cr, Edge::RIGHT);
		edges[i]->opaque = o ? true : false;
	}

	// Read in the number of cells
	if ( fscanf(f, "%d", &num_cells) != 1 )
		throw new MazeException("Maze: Couldn't read number of cells");


	// Read in all cells
	cells = new Cell*[num_cells];
	for ( i = 0 ; i < num_cells ; i++ )	{
		int epx, epy, emx, emy;
		if ( fscanf(f, "%d %d %d %d", &epx, &epy, &emx, &emy) != 4 ){
			sprintf(err_string, "Maze: Couldn't read cell number %d", i);
			throw new MazeException(err_string);
		}
		cells[i] = new Cell(i, epx >= 0 ? edges[epx] : NULL,
									epy >= 0 ? edges[epy] : NULL,
									emx >= 0 ? edges[emx] : NULL,
									emy >= 0 ? edges[emy] : NULL);
		if ( cells[i]->edges[0] ) {
			if ( cells[i]->edges[0]->neighbors[0] == (Cell*)i )
				cells[i]->edges[0]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[0]->neighbors[1] == (Cell*)i )
				cells[i]->edges[0]->neighbors[1] = cells[i];
			else	{
				sprintf(err_string,
						  "Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[0]->index);
				throw new MazeException(err_string);
			}
		}

		if ( cells[i]->edges[1] )	{
			if ( cells[i]->edges[1]->neighbors[0] == (Cell*)i )
				cells[i]->edges[1]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[1]->neighbors[1] == (Cell*)i )
				cells[i]->edges[1]->neighbors[1] = cells[i];
			else {
				sprintf(err_string,
							"Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[1]->index);
				throw new MazeException(err_string);
			}
		}
		if ( cells[i]->edges[2] ) {
			if ( cells[i]->edges[2]->neighbors[0] == (Cell*)i )
				cells[i]->edges[2]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[2]->neighbors[1] == (Cell*)i )
				cells[i]->edges[2]->neighbors[1] = cells[i];
			else	{
				sprintf(err_string,
							"Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[2]->index);
				throw new MazeException(err_string);
			}
		}
		if ( cells[i]->edges[3] ) {
			if ( cells[i]->edges[3]->neighbors[0] == (Cell*)i )
				cells[i]->edges[3]->neighbors[0] = cells[i];
			else if ( cells[i]->edges[3]->neighbors[1] == (Cell*)i )
				cells[i]->edges[3]->neighbors[1] = cells[i];
			else	{
				sprintf(err_string,
							"Maze: Cell %d not one of edge %d's neighbors",
							i, cells[i]->edges[3]->index);
				throw new MazeException(err_string);
			}
		}
	}

	if ( fscanf(f, "%g %g %g %g %g",
					 &(viewer_posn[X]), &(viewer_posn[Y]), &(viewer_posn[Z]),
					 &(viewer_dir), &(viewer_fov)) != 5 )
		throw new MazeException("Maze: Error reading view information.");

	// Some edges have no neighbor on one side, so be sure to set their
	// pointers to NULL. (They were set at -1 by the save/load process.)
	for ( i = 0 ; i < num_edges ; i++ )	{
		if ( edges[i]->neighbors[0] == (Cell*)-1 )
			edges[i]->neighbors[0] = NULL;
		if ( edges[i]->neighbors[1] == (Cell*)-1 )
			edges[i]->neighbors[1] = NULL;
	}

	fclose(f);

	Set_Extents();

	// Figure out which cell the viewer is in, starting off by guessing the
	// 0th cell.
	Find_View_Cell(cells[0]);

	frame_num = 0;
}


//**********************************************************************
//*清空記憶體占用
// * Destructor must free all the memory allocated.
//======================================================================
Maze::
~Maze(void)
//======================================================================
{
	int i;

	for ( i = 0 ; i < num_vertices ; i++ )
		delete vertices[i];
	delete[] vertices;

	for ( i = 0 ; i < num_edges ; i++ )
		delete edges[i];
	delete[] edges;

	for ( i = 0 ; i < num_cells ; i++ )
		delete cells[i];
	delete[] cells;
}


//**********************************************************************
//
// * Randomly generate the edge's opaque and transparency for an empty maze
//======================================================================
void Maze::
Build_Connectivity(const int num_x, const int num_y,
                   const float sx, const float sy)
//======================================================================
{
	int	i, j, k;
	int edge_i;

	// Ugly code to allocate all the memory for a new maze and to associate
	// edges with vertices and faces with edges.

	// Allocate and position the vertices.
	num_vertices = ( num_x + 1 ) * ( num_y + 1 );
	vertices = new Vertex*[num_vertices];
	k = 0;
	for ( i = 0 ; i < num_y + 1 ; i++ ) {
		for ( j = 0 ; j < num_x + 1 ; j++ )	{
			vertices[k] = new Vertex(k, j * sx, i * sy);
			k++;
		}
	}

	// Allocate the edges, and associate them with their vertices.
	// Edges in the x direction get the first num_x * ( num_y + 1 ) indices,
	// edges in the y direction get the rest.
	num_edges = (num_x+1)*num_y + (num_y+1)*num_x;
	edges = new Edge*[num_edges];
	k = 0;
	for ( i = 0 ; i < num_y + 1 ; i++ ) {
		int row = i * ( num_x + 1 );
		for ( j = 0 ; j < num_x ; j++ ) {
			int vs = row + j;
			int ve = row + j + 1;
			edges[k] = new Edge(k, vertices[vs], vertices[ve],
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f);
			k++;
		}
	}

	edge_i = k;
	for ( i = 0 ; i < num_y ; i++ ) {
		int row = i * ( num_x + 1 );
		for ( j = 0 ; j < num_x + 1 ; j++ )	{
			int vs = row + j;
			int ve = row + j + num_x + 1;
			edges[k] = new Edge(k, vertices[vs], vertices[ve],
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f,
			rand() / (float)RAND_MAX * 0.5f + 0.25f);
			k++;
		}
	}

	// Allocate the cells and associate them with their edges.
	num_cells = num_x * num_y;
	cells = new Cell*[num_cells];
	k = 0;
	for ( i = 0 ; i < num_y ; i++ ) {
		int row_x = i * ( num_x + 1 );
		int row_y = i * num_x;
		for ( j = 0 ; j < num_x ; j++ )	{
			int px = edge_i + row_x + 1 + j;
			int py = row_y + j + num_x;
			int mx = edge_i + row_x + j;
			int my = row_y + j;
			cells[k] = new Cell(k, edges[px], edges[py], edges[mx], edges[my]);
			edges[px]->Add_Cell(cells[k], Edge::LEFT);
			edges[py]->Add_Cell(cells[k], Edge::RIGHT);
			edges[mx]->Add_Cell(cells[k], Edge::RIGHT);
			edges[my]->Add_Cell(cells[k], Edge::LEFT);
			k++;
		}
	}
}


//**********************************************************************
//
// * Add edges from cell to the set that are available for removal to
//   grow the maze.
//======================================================================
static void
Add_To_Available(Cell *cell, int *available, int &num_available)
//======================================================================
{
	int i, j;

	// Add edges from cell to the set that are available for removal to
	// grow the maze.

	for ( i = 0 ; i < 4 ; i++ ){
		Cell    *neighbor = cell->edges[i]->Neighbor(cell);

		if ( neighbor && ! neighbor->counter )	{
			int candidate = cell->edges[i]->index;
			for ( j = 0 ; j < num_available ; j++ )
				if ( candidate == available[j] ) {
					printf("Breaking early\n");
					break;
			}
			if ( j == num_available )  {
				available[num_available] = candidate;
				num_available++;
			}
		}
	}

	cell->counter = 1;
}


//**********************************************************************
//
// * Grow a maze by removing candidate edges until all the cells are
//   connected. The edges are not actually removed, they are just made
//   transparent.
//======================================================================
void Maze::
Build_Maze()
//======================================================================
{
	Cell    *to_expand;
	int     index;
	int     *available = new int[num_edges];
	int     num_available = 0;
	int	    num_visited;
	int	    i;

	srand(time(NULL));

	// Choose a random starting cell.
	index = (int)floor((rand() / (float)RAND_MAX) * num_cells);
	to_expand = cells[index];
	Add_To_Available(to_expand, available, num_available);
	num_visited = 1;

	// Join cells up by making edges opaque.
	while ( num_visited < num_cells && num_available > 0 ) {
		int ei;

		index = (int)floor((rand() / (float)RAND_MAX) * num_available);
		to_expand = NULL;

		ei = available[index];

		if ( edges[ei]->neighbors[0] && 
			 !edges[ei]->neighbors[0]->counter )
			to_expand = edges[ei]->neighbors[0];
		else if ( edges[ei]->neighbors[1] && 
			 !edges[ei]->neighbors[1]->counter )
			to_expand = edges[ei]->neighbors[1];

		if ( to_expand ) {
			edges[ei]->opaque = false;
			Add_To_Available(to_expand, available, num_available);
			num_visited++;
		}

		available[index] = available[num_available-1];
		num_available--;
	}

	for ( i = 0 ; i < num_cells ; i++ )
		cells[i]->counter = 0;
}


//**********************************************************************
//
// * Go through all the vertices looking for the minimum and maximum
//   extents of the maze.
//======================================================================
void Maze::
Set_Extents(void)
//======================================================================
{
	int i;

	min_xp = vertices[0]->posn[Vertex::X];
	max_xp = vertices[0]->posn[Vertex::X];
	min_yp = vertices[0]->posn[Vertex::Y];
	max_yp = vertices[0]->posn[Vertex::Y];
	for ( i = 1 ; i < num_vertices ; i++ ) {
		if ( vertices[i]->posn[Vertex::X] > max_xp )
			 max_xp = vertices[i]->posn[Vertex::X];
		if ( vertices[i]->posn[Vertex::X] < min_xp )
			 min_xp = vertices[i]->posn[Vertex::X];
		if ( vertices[i]->posn[Vertex::Y] > max_yp )
			 max_yp = vertices[i]->posn[Vertex::Y];
		if ( vertices[i]->posn[Vertex::Y] < min_yp )
			 min_yp = vertices[i]->posn[Vertex::Y];
    }
}


//**********************************************************************
// 找出在視線範圍的cell
// * Figure out which cell the view is in, using seed_cell as an
//   initial guess. This procedure works by repeatedly checking
//   whether the viewpoint is in the current cell. If it is, we're
//   done. If not, Point_In_Cell returns in new_cell the next cell
//   to test. The new cell is the one on the other side of an edge
//   that the point is "outside" (meaning that it might be inside the
//   new cell).
//======================================================================
void Maze::
Find_View_Cell(Cell *seed_cell)
//======================================================================
{
	Cell    *new_cell;

	// 
	while ( ! ( seed_cell->Point_In_Cell(viewer_posn[X], viewer_posn[Y],
													 viewer_posn[Z], new_cell) ) ) {
		if ( new_cell == 0 ) {
			// The viewer is outside the top or bottom of the maze.
			throw new MazeException("Maze: View not in maze\n");
		}

		seed_cell = new_cell;
    }
    
    view_cell = seed_cell;
}


//**********************************************************************
//
// * Move the viewer's position. This method will do collision detection
//   between the viewer's location and the walls of the maze and prevent
//   the viewer from passing through walls.
//======================================================================
void Maze::
Move_View_Posn(const float dx, const float dy, const float dz)
//======================================================================
{
	Cell    *new_cell;
	float   xs, ys, zs, xe, ye, ze;

	// Move the viewer by the given amount. This does collision testing to
	// prevent walking through walls. It also keeps track of which cells the
	// viewer is in.

	// Set up a line segment from the start to end points of the motion.
	xs = viewer_posn[X];
	ys = viewer_posn[Y];
	zs = viewer_posn[Z];
	xe = xs + dx;
	ye = ys + dy;
	ze = zs + dz;

	// Fix the z to keep it in the maze.
	if ( ze > 1.0f - BUFFER )
		ze = 1.0f - BUFFER;
	if ( ze < BUFFER - 1.0f )
		ze = BUFFER - 1.0f;

	// Clip_To_Cell clips the motion segment to the view_cell if the
	// segment intersects an opaque edge. If the segment intersects
	// a transparent edge (through which it can pass), then it clips
	// the motion segment so that it _starts_ at the transparent edge,
	// and it returns the cell the viewer is entering. We keep going
	// until Clip_To_Cell returns NULL, meaning we've done as much of
	// the motion as is possible without passing through walls.
	while ( ( new_cell = view_cell->Clip_To_Cell(xs, ys, xe, ye, BUFFER) ) )
		view_cell = new_cell;

	// The viewer is at the end of the motion segment, which may have
	// been clipped.
	viewer_posn[X] = xe;
	viewer_posn[Y] = ye;
	viewer_posn[Z] = ze;
}

//**********************************************************************
//
// * Set the viewer's location 
//======================================================================
void Maze::
Set_View_Posn(float x, float y, float z)
//======================================================================
{
	// First make sure it's in some cell.
	// This assumes that the maze is rectangular.
	if ( x < min_xp + BUFFER )
		x = min_xp + BUFFER;
	if ( x > max_xp - BUFFER )
		x = max_xp - BUFFER;
	if ( y < min_yp + BUFFER )
		y = min_yp + BUFFER;
	if ( y > max_yp - BUFFER )
		y = max_yp - BUFFER;
	if ( z < -1.0f + BUFFER )
		z = -1.0f + BUFFER;
	if ( z > 1.0f - BUFFER )
		z = 1.0f - BUFFER;

	viewer_posn[X] = x;
	viewer_posn[Y] = y;
	viewer_posn[Z] = z;

	// Figure out which cell we're in.
	Find_View_Cell(cells[0]);
}


//**********************************************************************
//
// * Set the angle in which the viewer is looking.
//======================================================================
void Maze::
Set_View_Dir(const float d)
//======================================================================
{
	viewer_dir = d;
}


//**********************************************************************
//
// * Set the horizontal field of view.
//======================================================================
void Maze::
Set_View_FOV(const float f)
//======================================================================
{
	viewer_fov = f;
}


//**********************************************************************
//
// * Draws the map view of the maze. It is passed the minimum and maximum
//   corners of the window in which to draw.
//======================================================================
void Maze::
Draw_Map(int min_x, int min_y, int max_x, int max_y)
//======================================================================
{
	int	    height;
	float   scale_x, scale_y, scale;
	int	    i;

	// Figure out scaling factors and the effective height of the window.
	scale_x = ( max_x - min_x - 10 ) / ( max_xp - min_xp );
	scale_y = ( max_y - min_y - 10 ) / ( max_yp - min_yp );
	scale = scale_x > scale_y ? scale_y : scale_x;
	height = (int)ceil(scale * ( max_yp - min_yp ));

	min_x += 5;
	min_y += 5;

	// Draw all the opaque edges.
	for ( i = 0 ; i < num_edges ; i++ )
		if ( edges[i]->opaque )	{
			float   x1, y1, x2, y2;

			x1 = edges[i]->endpoints[Edge::START]->posn[Vertex::X];
			y1 = edges[i]->endpoints[Edge::START]->posn[Vertex::Y];
			x2 = edges[i]->endpoints[Edge::END]->posn[Vertex::X];
			y2 = edges[i]->endpoints[Edge::END]->posn[Vertex::Y];

			fl_color((unsigned char)floor(edges[i]->color[0] * 255.0),
					 (unsigned char)floor(edges[i]->color[1] * 255.0),
					 (unsigned char)floor(edges[i]->color[2] * 255.0));
			fl_line_style(FL_SOLID);
			fl_line(min_x + (int)floor((x1 - min_xp) * scale),
					  min_y + height - (int)floor((y1 - min_yp) * scale),
					  min_x + (int)floor((x2 - min_xp) * scale),
					  min_y + height - (int)floor((y2 - min_yp) * scale));
		}
}


void Maze::
Draw_Wall(const float start[2], const float end[2], const float color[3], float aspect)
{
	float viewer_pos[3] = { viewer_posn[Y], 0.0f, viewer_posn[X] };
	float edge0[3] = { start[Y],0.0f,start[X] };
	float edge1[3] = { end[Y],0.0f,end[X] };

	float dots[4][4] = {
		{ edge0[X], 1.0f, edge0[Z],1.0f },
		{ edge1[X], 1.0f, edge1[Z],1.0f },
		{ edge1[X], -1.0f, edge1[Z],1.0f },
		{ edge0[X], -1.0f, edge0[Z],1.0f }
	};

	for (int i = 0; i < 4; i++) {
		gluLookAt__(dots[i], viewer_pos[Maze::X], viewer_pos[Maze::Y], viewer_pos[Maze::Z],
			viewer_pos[Maze::X] + sin(Maze::To_Radians(viewer_dir)),
			viewer_pos[Maze::Y],
			viewer_pos[Maze::Z] + cos(Maze::To_Radians(viewer_dir)),
			0.0, 1.0, 0.0);
		gluPerspective__(dots[i], viewer_fov, aspect, 0.01f, 200.0f);
		ToScreen(dots[i]);
	}

	glBegin(GL_QUADS);
	glColor3fv(color);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			dots[i][j] /= dots[i][3];
		}
		glVertex2f(dots[i][0], dots[i][1]);
	}
	glEnd();
	
}


void Maze::
Draw_Cell(Cell *viewcell, LineSeg L, LineSeg R, float aspect) {
	float Near = 0.01f;
	float Far = 200.0f;
	viewcell->foot_print = true;
	float percent_L, percent_R;
	//測試 - 輸出全部
	/*for (int i = 0; i < 4; i++) {
		LineSeg edge_i = LineSeg(viewcell->edges[i]);
		Draw_Wall(edge_i.start, edge_i.end, viewcell->edges[i]->color, aspect);
	}*/

	for (int i = 0; i < 4; i++) {
		bool r_seg = false;
		bool seg_l = false;
		//edges[i] 的線段 start[X],start[Y],end[X],end[Y]
		LineSeg edge_i = LineSeg(viewcell->edges[i]);

		//到牆上的比例 若在0~1間代表有交點
		percent_L = L.Cross_Param(edge_i);
		percent_R = R.Cross_Param(edge_i);
		
		//若edge[i]的延伸和Frustum沒有交點則處理下個邊。
		if (!((percent_L > 0 && percent_L < 1) || (percent_R > 0 && percent_R < 1)))continue;
		
		////edge[i]的延伸與Frustum_L有交點 
		if (percent_L < 1 && percent_L>0) {

			//起點在Frustum_L的左邊
			if (L.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::LEFT) {
				//終點也在Frustum_L的左邊，代表不在視野範圍內，不畫。
				if (L.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::LEFT)continue;
				else {
					//終點不在左邊，更新起點位置。
					edge_i.start[X] = L.start[X] + (L.end[X] - L.start[X]) * percent_L;
					edge_i.start[Y] = L.start[Y] + (L.end[Y] - L.start[Y]) * percent_L;
				}

			}
			//起點在Frustum_L的右邊
			else if (L.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::RIGHT) {
				//終點在Frutstum_L的左邊，需clip。
				if (L.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::LEFT) {
					edge_i.end[X] = L.start[X] + (L.end[X] - L.start[X]) * percent_L;
					edge_i.end[Y] = L.start[Y] + (L.end[Y] - L.start[Y]) * percent_L;
				}
				else if (R.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::RIGHT) {
					if (!(R.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::LEFT || R.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::LEFT)) {
						//printf("LR_seg_continue\n");
						continue;
					}
				}
			}
		}
		////edge[i]的延伸與Frustum_R有交點 
		if (percent_R < 1 && percent_R>0) {
			//起點在Frustum_R右邊
			if (R.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::RIGHT) {
				//終點在Frustum_R右邊，不顯示。
				if (R.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::RIGHT)continue;
				//終點在Frustum_R左邊，做clip。
				edge_i.start[X] = R.start[X] + (R.end[X] - R.start[X]) * percent_R;
				edge_i.start[Y] = R.start[Y] + (R.end[Y] - R.start[Y]) * percent_R;
			}
			//起點在Frustum_R左邊
			else if (R.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::LEFT) {
				//終點在Frustum_R右邊，做clip。
				if (R.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::RIGHT) {
					edge_i.end[X] = R.start[X] + (R.end[X] - R.start[X]) * percent_R;
					edge_i.end[Y] = R.start[Y] + (R.end[Y] - R.start[Y]) * percent_R;
				}
				else if (L.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::LEFT) {
					if (!(L.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::RIGHT || L.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::RIGHT)) {
						//printf("LR_seg_continue\n");
						continue;
					}
				}
			}

		}

		//若牆壁為非透明，畫牆。
		if (viewcell->edges[i]->opaque ) {
			//if (seg_l || r_seg)continue;
			//std::cout << "draw:(" << edge_i.start[X] << ", " << edge_i.start[Y] << "),(" << edge_i.end[X] << ", " << edge_i.end[Y] << ") frustum:(" << L.end[X] << ", " << L.end[Y] << "),(" <<R.end[X] << ", " << R.end[Y] <<")\n";
			Draw_Wall(edge_i.start, edge_i.end, viewcell->edges[i]->color, aspect);
		}
		//若牆壁為透明，將Frustum更新為該線段。
		else {
			if (viewcell->edges[i]->Neighbor(viewcell)->foot_print==false) {
				//利用座標方式將frustum更新
				float delta_xs = edge_i.start[X] - viewer_posn[X];
				float delta_ys = edge_i.start[Y] - viewer_posn[Y];
				float delta_xe = edge_i.end[X] - viewer_posn[X];
				float delta_ye = edge_i.end[Y] - viewer_posn[Y];
				float dis_to_start = sqrtf(delta_xs * delta_xs + delta_ys * delta_ys);
				float dis_to_end = sqrtf(delta_xe * delta_xe + delta_ye * delta_ye);

				//製作viewer_posn到edge_i的起點線段
				LineSeg S(
					viewer_posn[X] + Near * delta_xs / dis_to_start,
					viewer_posn[Y] + Near * delta_ys / dis_to_start,
					viewer_posn[X] + Far * delta_xs / dis_to_start,
					viewer_posn[Y] + Far * delta_ys / dis_to_start
				);

				//製作viewer_posn到edge_i的終點線段
				LineSeg E(
					viewer_posn[X] + Near * delta_xe / dis_to_end,
					viewer_posn[Y] + Near * delta_ye / dis_to_end,
					viewer_posn[X] + Far * delta_xe / dis_to_end,
					viewer_posn[Y] + Far * delta_ye / dis_to_end
				);

				//設中點，判斷Start和End在左邊還是右邊。
				float edge_mid[2] = {
					(edge_i.start[X] + edge_i.end[X]) * 0.5,
					(edge_i.start[Y] + edge_i.end[Y]) * 0.5
				};
				LineSeg mid_line = LineSeg(viewer_posn[X], viewer_posn[Y], edge_mid[X], edge_mid[Y]);
				//判斷edge_i的中點在End線段的左邊還右邊，更新frustum更新。
				if (mid_line.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::LEFT && mid_line.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::RIGHT) {
					//std::cout << "ES" << edge_mid[X] << ", " << edge_mid[Y] << ")\n";
					//std::cout << "to draw:(" << edge_i.start[X] << ", " << edge_i.start[Y] << "),(" << edge_i.end[X] << ", " << edge_i.end[Y] << ")"<<"frustum:(" << S.end[X] << ", " << S.end[Y] << "),(" << E.end[X] << ", " << E.end[Y] << ")" << "\n";
					Draw_Cell(viewcell->edges[i]->Neighbor(viewcell), S, E, aspect);
				}
				else if (mid_line.Point_Side(edge_i.end[X], edge_i.end[Y]) == Edge::LEFT && mid_line.Point_Side(edge_i.start[X], edge_i.start[Y]) == Edge::RIGHT) {
					//std::cout << "SE" << edge_mid[X] << ", " << edge_mid[Y] << ")\n";
					//std::cout << "to draw:(" << edge_i.start[X] << ", " << edge_i.start[Y] << "),(" << edge_i.end[X] << ", " << edge_i.end[Y] << ")" << "frustum:(" << E.end[X] << ", " << E.end[Y] << "),(" << S.end[X] << ", " << S.end[Y] << ")" << "\n";
					Draw_Cell(viewcell->edges[i]->Neighbor(viewcell), E, S, aspect);
				}
			}
		}
	}
}


//**********************************************************************
//
// * Draws the first-person view of the maze. It is passed the focal distance.
//   THIS IS THE FUINCTION YOU SHOULD MODIFY.
//======================================================================
void Maze::
Draw_View(const float focal_dist, float aspect)
//======================================================================
{
	frame_num++;

	//###################################################################
	// TODO
	// The rest is up to you!
	//###################################################################

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float Near = 0.01f;
	float Far = 200.0f;
	//xs, ys, xe, ye
	LineSeg Frustum_R(
		viewer_posn[Maze::X] + Near * cos(Maze::To_Radians(viewer_dir - (viewer_fov) * 1 / 2)),
		viewer_posn[Maze::Y] + Near * sin(Maze::To_Radians(viewer_dir - (viewer_fov) * 1 / 2)),
		viewer_posn[Maze::X] + Far * cos(Maze::To_Radians(viewer_dir - (viewer_fov) * 1 / 2)),
		viewer_posn[Maze::Y] + Far * sin(Maze::To_Radians(viewer_dir - (viewer_fov) * 1 / 2))
	);

	LineSeg Frustum_L(
		viewer_posn[Maze::X] + Near * cos(Maze::To_Radians(viewer_dir + (viewer_fov) * 1 / 2)),
		viewer_posn[Maze::Y] + Near * sin(Maze::To_Radians(viewer_dir + (viewer_fov) * 1 / 2)),
		viewer_posn[Maze::X] + Far * cos(Maze::To_Radians(viewer_dir + (viewer_fov) * 1 / 2)),
		viewer_posn[Maze::Y] + Far * sin(Maze::To_Radians(viewer_dir + (viewer_fov) * 1 / 2))
	);
	for (int i = 0; i < num_cells; i++)cells[i]->foot_print = false;
	//std::cout << "#####start#####\n";
	Draw_Cell(view_cell, Frustum_L, Frustum_R,aspect);
	//std::cout << "#####end#####\n";
}



//**********************************************************************
//
// * Draws the frustum on the map view of the maze. It is passed the
//   minimum and maximum corners of the window in which to draw.
//======================================================================
void Maze::
Draw_Frustum(int min_x, int min_y, int max_x, int max_y)
//======================================================================
{
	int	  height;
	float   scale_x, scale_y, scale;
	float   view_x, view_y;

	// Draws the view frustum in the map. Sets up all the same viewing
	// parameters as draw().
	scale_x	= ( max_x - min_x - 10 ) / ( max_xp - min_xp );
	scale_y	= ( max_y - min_y - 10 ) / ( max_yp - min_yp );
	scale		= scale_x > scale_y ? scale_y : scale_x;
	height	= (int)ceil(scale * ( max_yp - min_yp ));

	min_x += 5;
	min_y += 5;

	view_x = ( viewer_posn[X] - min_xp ) * scale;
	view_y = ( viewer_posn[Y] - min_yp ) * scale;
	fl_line(min_x + (int)floor(view_x + 
			  cos(To_Radians(viewer_dir+viewer_fov / 2.0)) * scale),
			  min_y + height- 
			  (int)floor(view_y + 
							 sin(To_Radians(viewer_dir+viewer_fov / 2.0)) * 
							 scale),
				min_x + (int)floor(view_x),
				min_y + height - (int)floor(view_y));
	fl_line(min_x + (int)floor(view_x + 
										cos(To_Radians(viewer_dir-viewer_fov / 2.0))	* 
										scale),
				min_y + height- 
				(int)floor(view_y + sin(To_Radians(viewer_dir-viewer_fov / 2.0)) *
				scale),
				min_x + (int)floor(view_x),
				min_y + height - (int)floor(view_y));
	}



//**********************************************************************
//
// * Draws the viewer's cell and its neighbors in the map view of the maze.
//   It is passed the minimum and maximum corners of the window in which
//   to draw.
//======================================================================
void Maze::
Draw_Neighbors(int min_x, int min_y, int max_x, int max_y)
//======================================================================
{
	int	    height;
	float   scale_x, scale_y, scale;
	int	    i, j;

	// Draws the view cell and its neighbors in the map. This works
	// by drawing just the neighbor's edges if there is a neighbor,
	// otherwise drawing the edge. Every edge is shared, so drawing the
	// neighbors' edges also draws the view cell's edges.

	scale_x = ( max_x - min_x - 10 ) / ( max_xp - min_xp );
	scale_y = ( max_y - min_y - 10 ) / ( max_yp - min_yp );
	scale = scale_x > scale_y ? scale_y : scale_x;
	height = (int)ceil(scale * ( max_yp - min_yp ));

	min_x += 5;
	min_y += 5;

	for ( i = 0 ; i < 4 ; i++ )   {
		Cell	*neighbor = view_cell->edges[i]->Neighbor(view_cell);

		if ( neighbor ){
			for ( j = 0 ; j < 4 ; j++ ){
				Edge    *e = neighbor->edges[j];

				if ( e->opaque )	{
					float   x1, y1, x2, y2;

					x1 = e->endpoints[Edge::START]->posn[Vertex::X];
					y1 = e->endpoints[Edge::START]->posn[Vertex::Y];
					x2 = e->endpoints[Edge::END]->posn[Vertex::X];
					y2 = e->endpoints[Edge::END]->posn[Vertex::Y];

					fl_color((unsigned char)floor(e->color[0] * 255.0),
							  (unsigned char)floor(e->color[1] * 255.0),
							  (unsigned char)floor(e->color[2] * 255.0));
					fl_line_style(FL_SOLID);
					fl_line( min_x + (int)floor((x1 - min_xp) * scale),
							 min_y + height - (int)floor((y1 - min_yp) * scale),
							 min_x + (int)floor((x2 - min_xp) * scale),
							 min_y + height - (int)floor((y2 - min_yp) * scale));
				}
			}
		}
		else {
			Edge    *e = view_cell->edges[i];

			if ( e->opaque ){
				float   x1, y1, x2, y2;

				x1 = e->endpoints[Edge::START]->posn[Vertex::X];
				y1 = e->endpoints[Edge::START]->posn[Vertex::Y];
				x2 = e->endpoints[Edge::END]->posn[Vertex::X];
				y2 = e->endpoints[Edge::END]->posn[Vertex::Y];

				fl_color((unsigned char)floor(e->color[0] * 255.0),
							 (unsigned char)floor(e->color[1] * 255.0),
							 (unsigned char)floor(e->color[2] * 255.0));
				fl_line_style(FL_SOLID);
				fl_line(min_x + (int)floor((x1 - min_xp) * scale),
							min_y + height - (int)floor((y1 - min_yp) * scale),
							min_x + (int)floor((x2 - min_xp) * scale),
							min_y + height - (int)floor((y2 - min_yp) * scale));
			 }
		}
	}
}


//**********************************************************************
//
// * Save the maze to a file of the given name.
//======================================================================
bool Maze::
Save(const char *filename)
//======================================================================
{
	FILE    *f = fopen(filename, "w");
	int	    i;

	// Dump everything to a file of the given name. Returns false if it
	// couldn't open the file. True otherwise.

	if ( ! f )  {
		return false;
   }

	fprintf(f, "%d\n", num_vertices);
	for ( i = 0 ; i < num_vertices ; i++ )
		fprintf(f, "%g %g\n", vertices[i]->posn[Vertex::X],
			      vertices[i]->posn[Vertex::Y]);

		fprintf(f, "%d\n", num_edges);
	for ( i = 0 ; i < num_edges ; i++ )
	fprintf(f, "%d %d %d %d %d %g %g %g\n",
				edges[i]->endpoints[Edge::START]->index,
				edges[i]->endpoints[Edge::END]->index,
				edges[i]->neighbors[Edge::LEFT] ?
				edges[i]->neighbors[Edge::LEFT]->index : -1,
				edges[i]->neighbors[Edge::RIGHT] ?
				edges[i]->neighbors[Edge::RIGHT]->index : -1,
				edges[i]->opaque ? 1 : 0,
				edges[i]->color[0], edges[i]->color[1], edges[i]->color[2]);

	fprintf(f, "%d\n", num_cells);
	for ( i = 0 ; i < num_cells ; i++ )
		fprintf(f, "%d %d %d %d\n",
					cells[i]->edges[0] ? cells[i]->edges[0]->index : -1,
					cells[i]->edges[1] ? cells[i]->edges[1]->index : -1,
					cells[i]->edges[2] ? cells[i]->edges[2]->index : -1,
					cells[i]->edges[3] ? cells[i]->edges[3]->index : -1);

	   fprintf(f, "%g %g %g %g %g\n",
					viewer_posn[X], viewer_posn[Y], viewer_posn[Z],
					viewer_dir, viewer_fov);

	fclose(f);

	return true;
}

