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

        bool operator==(const Position& p) {
            return p.row == this->row && p.col == this->col;
        }

        bool operator!=(const Position& p) {
            return !(*this == p);
        }

        int offset();
        float direction();
        void print(int bluetooth=0);

        int row;
        int col;
};


class Maze {
    public:
        Maze();

        void initializeMaze();
        void floodMaze();
        void addWalls(float angle, long leftDiag, long front, long rightDiag);
        void printMaze();
        Position chooseNextCell(Position pos);
        void reset();
        void updatePosition(Position p);
        void setBoundaryWalls();

        void printWallsCell(Position p);

        /** Constants **/

        /* Current cell in the maze of the robot */
        Position currPos;
        Position goalPos;
        Position startPos;

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