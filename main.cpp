#include <iostream>

#include "Manager/Manager.h"


int main() {
    cout << "--------------------" << endl;
    cout << "|     Welcome!     |" << endl;
    cout << "--------------------" << endl;
    cout << "| Choose the Graph |" << endl;
    cout << "| 1 - Small        |" << endl;
    cout << "| 2 - Large        |" << endl;
    cout << "--------------------" << endl;

    string choice;
    cin >> choice;

    while (choice != "1" && choice != "2") {
        cout << "Invalid Choice! (Type 1 or 2)" << endl;
        cin >> choice;
    }

    Manager manager = Manager(choice);
    return 0;
}
