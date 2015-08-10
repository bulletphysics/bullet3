#ifndef TUTORIAL_H
#define TUTORIAL_H

enum EnumTutorialTypes
{
	TUT_VELOCITY=0,
	TUT_ACCELERATION,
	TUT_COLLISION,
	TUT_SOLVE_CONTACT_CONSTRAINT,
};

class	CommonExampleInterface*    TutorialCreateFunc(struct CommonExampleOptions& options);

#endif //TUTORIAL_H
