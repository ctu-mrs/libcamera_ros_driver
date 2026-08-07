#include <iostream>
#include <vector>

template <typename T>
struct is_vector : std::false_type
{
};

template <typename T>
struct is_vector<std::vector<T>> : std::true_type
{
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
  os << "[ ";
  for (const auto& e : vec)
  {
    os << e << (&e == &vec.back() ? "" : is_vector<std::decay_t<decltype(e)>>::value ? "\n" : ", ");
  }
  os << " ]";
  return os;
}
