/*
    Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LISTVIEWWITHDELETE_H
#define LISTVIEWWITHDELETE_H

#include <QListView>
#include <QKeyEvent>

class ListViewWithDelete : public QListView
{
    Q_OBJECT

public:
    explicit ListViewWithDelete(QWidget *parent = nullptr);
    
signals:
    void deletePressed();
    
protected:
    void keyPressEvent(QKeyEvent *event) override;
};

#endif // LISTVIEWWITHDELETE_H
