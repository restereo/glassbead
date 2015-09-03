
// Устаканенная решетка
vector<Vec2f> _hbars;
vector<Vec2f> _vbars;

// функция для построения решетки по отфильтрованным линиям
// EXPERIMENTAL
vector<Vec2f> setBars(vector<Vec2f> hlines2) { // , vector<Vec2f> vlines2

  // предполагаем, что верхняя и нижняя линии распознаны правильно и достроим решетку по ним, посчитав средний dr

  vector<float> _h_drs;
  float _prev_r;
  _prev_r = 0.0;

  for (int i = 0; i < hlines2.size(); ++i) {

    float r = hlines2[i][0];
    _h_drs.push_back(abs(r - _prev_r));
  }

  float m_dr = (_h_drs);
  printf("median dr: %1.f\n", m_dr);
  // sort(_h_drs.begin(), _h_drs.end());

  std::vector<int> _lines_to_add;
  std::vector<int> _lines_to_remove;

  float magic = 1.5;

  if (hlines2.size() < 21) { // надо добавить линий

    for (int i = 0; i < _h_drs.size(); ++i) {

      if (_h_drs[i] > m_dr * magic) {

        _lines_to_add.push_back(i);
      }
    }
  } else { // надо убрать лишние линии
    for (int i = 0; i < _h_drs.size(); ++i) {
      if (_h_drs[i] < m_dr/magic) {

        _lines_to_remove.push_back(i);
      }
    }
  }

  for (int i = 0; i < _lines_to_add.size(); ++i) {
    hlines2.push_back(Vec2f(hlines2[i][0] + m_dr, hlines2[i][1]));
  }

  for (int i = 0; i < _lines_to_remove.size(); ++i) {
    hlines2.erase (hlines2.begin()+i);
  }

  sort(hlines2.begin(), hlines2.end(), waytosort);

  return hlines2;

}