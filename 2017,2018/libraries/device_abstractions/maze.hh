#ifndef maze_hh
#define maze_hh

class Position {
    public:
        Position(int r, int c);

        Position operator-(const Position& p) {
            Position pos(0, 0);
            pos.row = this->row - p.row;
            pos.col = this->col - p.col;
            return pos;
        }

        int offset();
        float direction();
        void print();

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
        void updatePosition(Position p);
        void setBoundaryWalls();

        // current position
        Position currPos;
        /* Global counter, keeps track of run number to set speed and
         * destination cell */
        int counter = 0;

    private:
        /* Stores flood-fill information for each cell in the map of the maze */
        unsigned char cellMap[256];

        /* Stores a map where each entry represents a cell in the maze and each
         * bit of the first four bits is set if there is a wall at the
         * corresponding location */
        unsigned char wallMap[256];
};

#endif
