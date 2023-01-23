#include <mutex>

namespace m_rosuam::gui
{

	template <typename T>
	class AtomicVariable
	{
	public:
		AtomicVariable() : value(T()) {}
		AtomicVariable(T initVal) : value(initVal) {}
		AtomicVariable(const AtomicVariable& other)
		{
			value = other.value;
			mut = other.mut;
		}
		~AtomicVariable() = default;
		
		T get() const
		{
			T returnVal;
			mut.lock();
			returnVal = value;
			mut.unlock();
			return returnVal;
		}
		
		operator T() const
		{
			return get();
		}
		
		void operator=(T newVal)
		{
			mut.lock();
			value = newVal;
			mut.unlock();
		}
		
	private:
		T value;
		mutable std::mutex mut;
	};

}
