import { Component, ViewChild, AfterViewInit, ChangeDetectionStrategy } from '@angular/core';
import { FormGroup } from '@angular/forms';
import { Observable } from 'rxjs';
import { NetworkEditComponent } from '../network-edit/network.edit.component';

@Component({
    selector: 'app-network',
    templateUrl: './network.component.html',
    styleUrls: ['./network.component.scss'],
    changeDetection: ChangeDetectionStrategy.Eager,
    standalone: false
})
export class NetworkComponent implements AfterViewInit {
  form$!: Observable<FormGroup | null>;

  @ViewChild(NetworkEditComponent) networkEditComponent!: NetworkEditComponent;

  constructor() {}

  ngAfterViewInit() {
    this.form$ = this.networkEditComponent.form$;
  }
}
