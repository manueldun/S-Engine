#include "Interfaces.h"

void Subject::addObserver(Observer *observer) {
  m_observers.push_back(observer);
}

void Subject::notify(const Event &event,
                     const std::variant<void *, std::string> &data) {

  for (Observer *observer : m_observers) {
    observer->onNotify(event, data);
  }
}
