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

#include "listviewwithdelete.h"

ListViewWithDelete::ListViewWithDelete(QWidget *parent) : QListView(parent)
{
}

void ListViewWithDelete::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Delete) {
        emit deletePressed();
        event->accept();
    } else {
        QListView::keyPressEvent(event);
    }
}
