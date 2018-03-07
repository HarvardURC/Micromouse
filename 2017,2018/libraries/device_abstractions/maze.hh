#ifndef maze_hh
#define maze_hh

class Position {
    public:
        Position(int r, int c);

        int offset();

        int row;
        int col;
};


class Maze {
    public:
        // constructor
        Maze();

        void initializeMaze();
        void floodMaze();
        void addWalls(float angle, long leftDiag, long front, long rightDiag);
        void printMaze();
        Position chooseNextCell();
        void resetPosition();
        void updatePosition(int row, int col);
        void setBoundaryWalls();

        // store the cell values
        unsigned char cellMap[256];
        // store the wall bits
        unsigned char wallMap[256];

        // current position
        Position currPos;
        // Global counter, keeps track of run number to set speed and destination cell
        int counter = 0;
};

#endif
