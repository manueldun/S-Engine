#pragma once
#include <list>
#include <memory>
#include <variant>
enum Event { OPEN_GLTF_FILE, CONTINUE_SIMULATION, STOP_SIMULATION };
class Observer {
public:
  virtual ~Observer() = default;
  virtual void onNotify(const Event &event,
                        const std::variant<std::string> &data) = 0;

private:
};
class Subject {
public:
  void addObserver(Observer *observer);

protected:
  void notify(const Event &event, const std::variant<std::string> &data);

  std::list<Observer *> m_observers;
};
