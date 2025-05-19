void skills();
void sixRingBlueLeft();
void sixRingRedRight();
void blueRight();
void redRightElim();
void blueLeftElim();
void redLeftElim();
void blueRightElim();
void redLeft();
void AWPRed();
void AWPBlue();
void qualRedRight();
void qualBlueLeft();
void qualRedRightTanner();
void testingPID();
void sixRingRedLeftElim();
void eightRingBlueRight();
void rushPlusSixRingRedRight();



void sixRingRedRightElims();

void sixRingBlueLeftElims();


class PIDTWO {
    public:
        PIDTWO(double kP, double kI, double kD, double skP, double skI, double skD, double windupRange = 0, bool signFlipReset = false);
        float runLoop(float error);

    private:
        double kP;
        double kI;
        double kD;
        double skP;
        double skI;
        double skD;
        double windupRange;
        bool signFlipReset;
        float derivative;
        float integral;
        float prevError;
};