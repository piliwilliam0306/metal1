#pragma once

class RecvList
{
public:
	RecvList();
	~RecvList();
	void Add(void *item);
	void Delete(void *item);
	void DeleteAt(int index);
	int GetCount();
	void* GetItem(int index);
	bool Exists(void *item);
	int GetIndex(void *item);
private:

	int _count, _maxCount;
	void **_arr;
	void PrepareArray(int nextSize);
};

